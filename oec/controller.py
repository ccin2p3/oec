"""
oec.controller
~~~~~~~~~~~~~~
"""

import time
import logging
import selectors
from coax import InterfaceFeature, Poll, PollAck, KeystrokePollResponse, ReceiveTimeout, \
                 get_device_address
from coax.multiplexer import PORT_MAP_3299

from .device import address_commands, format_address, UnsupportedDeviceError
from .keyboard import Key
from .session import SessionDisconnectedError

class Controller:
    """The controller."""

    def __init__(self, interface, create_device, create_session):
        self.logger = logging.getLogger(__name__)

        self.interface = interface
        self.running = False

        self.create_device = create_device
        self.create_session = create_session

        self.devices = { }
        self.detatched_device_poll_queue = []

        self.session = None
        self.session_selector = None

        # Target time between POLL commands in seconds when a device is attached or
        # no device is attached.
        #
        # The attached poll period only applies in cases where the device responded
        # with TT/AR to the last poll - this is an effort to improve the keystroke
        # responsiveness.
        self.attached_poll_period = 1 / 15
        self.detatched_poll_period = 1 / 2

        self.poll_depth = 3

        self.last_attached_poll_time = None
        self.last_detatched_poll_time = None

    def run(self):
        """Run the controller."""
        self.running = True

        self.session_selector = selectors.DefaultSelector()

        while self.running:
            self._run_loop()

        self._terminate_session()

        self.session_selector.close()

        self.session_selector = None

        self.devices = { }

    def stop(self):
        """Stop the controller."""
        self.running = False

    def _run_loop(self):
        poll_delay = self._calculate_poll_delay()

        # If POLLing is delayed, handle the host output, otherwise just sleep.
        if poll_delay > 0:
            if self.session:
                self._update_session(poll_delay)
            else:
                time.sleep(poll_delay)

        # POLL devices.
        self._poll_attached_devices()
        self._poll_next_detatched_device()

    def _update_session(self, duration):
        try:
            update_count = 0

            while duration > 0:
                start_time = time.perf_counter()

                selected = self.session_selector.select(duration)

                if not selected:
                    break

                for (key, _) in selected:
                    session = key.fileobj

                    if session.handle_host():
                        update_count += 1

                duration -= (time.perf_counter() - start_time)

            if update_count > 0:
                self.session.render()
        except SessionDisconnectedError:
            self._handle_session_disconnected()

    def _start_session(self, device):
        self.session = self.create_session(device)

        self.session.start()

        self.session_selector.register(self.session, selectors.EVENT_READ)

    def _terminate_session(self):
        if not self.session:
            return

        self.session_selector.unregister(self.session)

        self.session.terminate()

        self.session = None

    def _handle_session_disconnected(self):
        self.logger.info('Session disconnected')

        device = self.session.terminal

        self._terminate_session()

        # Restart the session.
        self._start_session(device)

    def _poll_attached_devices(self):
        self.last_attached_poll_time = time.perf_counter()

        for device in self.devices.values():
            for _ in range(self.poll_depth):
                print('.')
                try:
                    poll_response = device.poll()
                except ReceiveTimeout:
                    self._handle_device_lost(device)
                    return

                if not poll_response:
                    break

                self._poll_ack(device.device_address)

                self._handle_poll_response(device, poll_response)

    def _poll_next_detatched_device(self):
        if self.last_detatched_poll_time is not None and (time.perf_counter() - self.last_detatched_poll_time) < self.detatched_poll_period:
            return

        self.last_detatched_poll_time = time.perf_counter()

        if not self.detatched_device_poll_queue:
            self.detatched_device_poll_queue = list(self._get_detatched_device_addresses())

        try:
            device_address = self.detatched_device_poll_queue.pop(0)
        except IndexError:
            return

        try:
            poll_response = self._poll(device_address)
        except ReceiveTimeout:
            return

        if poll_response:
            self._poll_ack(device_address)

        self._handle_device_found(device_address, poll_response)

    def _handle_device_found(self, device_address, poll_response):
        self.logger.info(f'Found device @ {format_address(self.interface, device_address)}')

        try:
            device = self.create_device(self.interface, device_address, poll_response)
        except UnsupportedDeviceError as error:
            self.logger.error(f'Unsupported device @ {format_address(self.interface, device_address)}: {error}')
            return

        device.setup()

        self.devices[device_address] = device

        self.logger.info(f'Attached device @ {format_address(self.interface, device_address)}')

        self._start_session(device)

    def _handle_device_lost(self, device):
        device_address = device.device_address

        self.logger.info(f'Lost device @ {format_address(self.interface, device_address)}')

        self._terminate_session()

        del self.devices[device_address]

        self.logger.info(f'Detached device @ {format_address(self.interface, device_address)}')

    def _handle_poll_response(self, device, poll_response):
        if isinstance(poll_response, KeystrokePollResponse):
            self._handle_keystroke_poll_response(device, poll_response)

    def _handle_keystroke_poll_response(self, terminal, poll_response):
        scan_code = poll_response.scan_code

        (key, modifiers, modifiers_changed) = terminal.keyboard.get_key(scan_code)

        if self.logger.isEnabledFor(logging.DEBUG):
            self.logger.debug((f'Keystroke detected: Scan Code = {scan_code}, '
                               f'Key = {key}, Modifiers = {modifiers}'))

        # Update the status line if modifiers have changed.
        if modifiers_changed:
            terminal.display.status_line.write_keyboard_modifiers(modifiers)

        if not key:
            return

        if key == Key.CURSOR_BLINK:
            terminal.display.toggle_cursor_blink()
        elif key == Key.ALT_CURSOR:
            terminal.display.toggle_cursor_reverse()
        elif key == Key.CLICKER:
            terminal.keyboard.toggle_clicker()
        elif self.session:
            self.session.handle_key(key, modifiers, scan_code)

            self.session.render()

    def _poll(self, device_address):
        return self.interface.execute(address_commands(device_address, Poll()))

    def _poll_ack(self, device_address):
        self.interface.execute(address_commands(device_address, PollAck()))

    def _calculate_poll_delay(self):
        if self.last_attached_poll_time is None:
            return 0

        return max((self.last_attached_poll_time + self.attached_poll_period) - time.perf_counter(), 0)

    def _get_detatched_device_addresses(self):
        attached_addresses = set(self.devices.keys())

        # The 3299 is transparent, but if there is at least one device attached to a 3299
        # port then we can assume there is a 3299 attached.
        is_3299_attached = any(attached_addresses.difference([None]))

        if InterfaceFeature.PROTOCOL_3299 not in self.interface.features:
            addresses = [None]
        elif is_3299_attached:
            addresses = PORT_MAP_3299
        else:
            addresses = [None, *PORT_MAP_3299]

        return filter(lambda address: address not in attached_addresses, addresses)
