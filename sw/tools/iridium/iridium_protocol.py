import re
import time
from twisted.internet import protocol, reactor

NO_CARRIER = 'NO CARRIER'
BUSY = 'BUSY'

class IridiumProtocol(protocol.Protocol):

  delimiter = '\r\n'        # Delemiter used in the iridium device
  serial_ctrl = False       # Do we use CTS and RTS?
  cpin1 = '1111'            # PIN code 1 for the SIM card
  cpin2 = '2222'            # PIN code 2 for the SIM card
  timeout = 3               # TImeout for receiving an OK or ERROR in seconds

  cb_console_write = None   # Console writing function
  cb_connected = None       # Connected callback function
  cb_configured = None      # Configured callback function
  cb_calling = None         # Calling callback function
  cb_registered = None      # Registered callback function
  cb_call_data = None       # Call Data received callback function

  last_cmd = ''             # Last command transmitted
  echo_enabled = True       # Whether echo is enabled
  in_call = False           # Are we currently in call
  recv_data = ''            # The received data until now
  recv_ok = False           # Whether we received an OK back
  recv_error = False        # Whether we received an ERROR back

  def __init__(self, cb_connected = None, cb_configured = None, cb_calling = None, cb_registered = None, cb_call_data = None):
    self.cb_connected = cb_connected
    self.cb_configured = cb_configured
    self.cb_calling = cb_calling
    self.cb_registered = cb_registered
    self.cb_call_data = cb_call_data

  def connectionMade(self):
    reactor.callInThread(self.configure)

    # Report connection has been made
    if self.cb_connected != None:
      self.cb_connected(True)

  def connectionLost(self, reason):
    # Report that connection is lost and configruation also(maybe)
    if self.cb_connected != None:
      self.cb_connected(False)
    if self.cb_configured != None:
      self.cb_configured(False)


  def configure(self):
    # Send the serial and echo settings
    if self.serial_ctrl:
      cfg_cmd = "ATE0 V1 X3"
    else:
      cfg_cmd = "ATE0 V1 X3"
    if not self.transmit_cmd_ok(cfg_cmd):
      if self.cb_configured != None:
        self.cb_configured(False)
      return
    self.echo_enabled = False

    # Set the data connection settings
    if not self.transmit_cmd_ok("AT+CBST=7,0,1"):
      if self.cb_configured != None:
        self.cb_configured(False)
      return

    # Report that the device has been configured
    if self.cb_configured != None:
      self.cb_configured(True)

  def set_console(self, console_write):
    self.console_write = console_write

  def register(self):
    self.transmit_cmd("AT+CPIN?")

  def get_csq(self):
    self.transmit_cmd("AT+CSQ?")

  def call(self, number):
    self.in_call = True
    self.transmit_cmd("ATD" + number)
    if self.cb_calling != None:
      self.cb_calling(True)

  # Wait for a OK receive
  def wait_for_ok(self):
    i = 0
    while (not self.recv_ok and not self.recv_error and i < (self.timeout/0.1)):
      time.sleep(0.1)
      i += 1

    # Report error if timeout
    if i >= (self.timeout/0.1) and self.console_write != None:
      reactor.callFromThread(self.console_write, "TIMEOUT (did not receive OK)\r\n")

    retn = self.recv_ok
    self.recv_ok = False
    self.recv_error = False
    return retn

  # Transmit a command and do a busy waiting for OK
  def transmit_cmd_ok(self, cmd):
    self.recv_ok = False
    self.recv_error = False
    self.transmit_cmd(cmd)

    return self.wait_for_ok()

  # Transmit a command
  def transmit_cmd(self, cmd):
    self.last_cmd = cmd
    if self.console_write != None:
      reactor.callFromThread(self.console_write, ">> " + cmd + "\r\n")
    self.transmit(cmd + self.delimiter)

  # Transmit data
  def transmit(self, data):
    reactor.callFromThread(self.transport.write, data)

  # Handle the 'OK' message
  def handle_ok(self, data):
    self.recv_ok = True

  # Handle the 'ERROR' message
  def handle_error(self, data):
    self.recv_error = True

  # Handle the 'CREG' message
  def handle_creg(self, data):
    if data == '001,001' and self.cb_registered != None:
      self.cb_registered(True)
    elif self.cb_registered != None:
      self.cb_registered(False)

  # Handle the 'CPIN' message
  def handle_cpin(self, data):
    reactor.callInThread(self._handle_cpin, data)

  # Handle the 'CPIN' message in thread
  def _handle_cpin(self,data):
    if not self.wait_for_ok():
      return

    if data == 'SIM PIN1':
      self.transmit_cmd_ok("AT+CPIN=\"%s\"" % self.cpin1)
    elif data == 'SIM PIN2':
      self.transmit_cmd_ok("AT+CPIN=\"%s\"" % self.cpin2)
    elif data == 'READY':
      self.transmit_cmd_ok("AT+CREG=1")
      self.transmit_cmd("AT+CREG?")
      return
    self.transmit_cmd("AT+CPIN?")

  # Hande data when we are in call (Bytes + possible Call ending)
  def handleInCall(self):
    # Check if we could have gotten an abort sequence
    if self.recv_data.strip() != '' and len(self.recv_data) < 50:
      # Check for NO Carrier or BUSY
      if NO_CARRIER.startswith(self.recv_data.strip()) or BUSY.startswith(self.recv_data.strip()):
        if self.recv_data[-len(self.delimiter):] == self.delimiter:
          self.in_call = False
          self.recv_data = ''
          if self.console_write != None:
            self.console_write("NO CARRIER\r\n")
          if self.cb_calling != None:
            self.cb_calling(False)
        return

      # Check if we got an +CR:, CONNECT and ignore
      if self.recv_data.strip().startswith("+CR:") or self.recv_data.strip().startswith("CONNECT"):
        if self.recv_data[-len(self.delimiter):] == self.delimiter:
          if self.console_write != None:
            self.console_write(self.recv_data.strip() + "\r\n")
          self.recv_data = ''
        return

    # Handle the in call data
    if self.console_write != None:
      self.console_write("< %s\r\n" % self.recv_data.encode("hex"))
    if self.cb_call_data != None:
      self.cb_call_data(self.recv_data)

    # ECHO for now (TODO remove)
    #reactor.callFromThread(self.transport.write, self.recv_data)
    self.recv_data = ''

  # Handle data when we are in terminal mode (String only)
  def handleInTerminal(self):
    # Check if we received a full line
    if self.recv_data[-len(self.delimiter):] != self.delimiter:
      return
    self.recv_data = self.recv_data.strip()

    # Check if we did not receive an empty line
    if self.recv_data == '':
      return

    # Check if echoing is enabled still, then ignore
    if self.echo_enabled and self.recv_data == self.last_cmd:
      self.recv_data = ''
      return

    # Split the information from the received line
    if self.console_write != None:
      self.console_write(self.recv_data + "\r\n")
    reg = re.match(r'([+]?)([A-Za-z0-9 ]+)[:]?[ ]?([A-Za-z0-9, ]*)', self.recv_data)
    #plus = reg.group(1)
    command = reg.group(2).lower()
    data = reg.group(3).strip()

    # Dispatch the handler
    handler = getattr(self, "handle_%s" % command, None)
    if handler:
      handler(data)

    # Reset the received data
    self.recv_data = ''

  # When we receive data over the serial device
  def dataReceived(self, data):
    self.recv_data += data

    if self.in_call:
      self.handleInCall() # We are currently calling
    else:
      self.handleInTerminal() # Not in call
