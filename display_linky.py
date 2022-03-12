#!/usr/bin/env python2.7
import Adafruit_Nokia_LCD as LCD

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import traceback
import serial
import threading
import Queue
import time

"""
Sample use as a Linky decoding library
import display_linky	# Import the library
s=display_linky.PhyDecoder(baudrate=9600, port="/dev/ttyUSB0")	# Enable the TIC phy decoder on serial port ttyUSB0 at baudrate 9600 ("TIC historique")
l=display_linky.TICLinkLayerDecoder(s)	# Create a Link layer decoder, fetching raw data from the previous PhyDecoder
tf=display_linky.TICFrames(l,standard_tic_mode=True)	# Create a TIC frame parser
for frame in tf:	# Forever loop returning each new frame as it is ready from the serial port
 print(frame)

"""

class TerminalColor:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class PhyDecoder:
	"""@brief TIC physical layer decoder, based on a Linux serial port connection
	"""
	DEFAULT_BAUDRATE = 1200
	DEFAULT_BITS_PER_SYMBOL = serial.SEVENBITS
	DEFAULT_PARITY = serial.PARITY_EVEN
	DEFAULT_STOP_BITS = serial.STOPBITS_ONE

	def __init__(self,
	             port,
	             baudrate=DEFAULT_BAUDRATE,
	             bits_per_symbol=DEFAULT_BITS_PER_SYMBOL,
	             parity=DEFAULT_PARITY,
	             stop_bits=DEFAULT_STOP_BITS):
		self._serial_port = serial.Serial(port=port,
		                                  baudrate=baudrate,
		                                  parity=parity,
		                                  stopbits=stop_bits,
		                                  bytesize=bits_per_symbol)

	def __del__(self):
		if self._serial_port is not None:
			if self._serial_port.is_open:
				self._serial_port.close()
				self._serial_port = None

	def get_next_incoming_bytes(self):
		assert self._serial_port is not None
		return self._serial_port.read_all()

class TICLinkLayerDecoder:
	"""@brief TIC Link layer decoder
	"""
	STX_BYTE = '\x02'
	ETX_BYTE = '\x03'

	def __init__(self, phy_decoder):
		"""@brief TIC Link kayer decoder

		@param phy_decoder An instance of type PhyDecoder that provides incoming bytes read from the serial port (possibliy by chunks)
		"""
		assert isinstance(phy_decoder, PhyDecoder)
		self._phy_decoder = phy_decoder
		self.initial_frame_sync = False
		self.incoming_buffer = ''

	def get_next_frame(self):
		self.incoming_buffer += self._phy_decoder.get_next_incoming_bytes()
		stx_pos = self.incoming_buffer.find(self.STX_BYTE)
		if stx_pos == -1:
			return None	# No sync
		else:
			if stx_pos != 0:
				self.incoming_buffer = self.incoming_buffer[stx_pos:]	# Keep only buffer from STX byte
				if self.initial_frame_sync:
					print('Warning: Lost synchronization')
			self.initial_frame_sync = True	# We are now in sync
		etx_pos = self.incoming_buffer.find(self.ETX_BYTE)
		if etx_pos == -1:
			return ''	# No full frame yet
		else:
			one_frame_buffer = self.incoming_buffer[1:etx_pos]	# We get rid of STX and ETX chars, this is our next frame
			self.incoming_buffer = self.incoming_buffer[etx_pos+1:]	# Keep in our internal buffer only the bytes after ETX
			return one_frame_buffer

class TICDataSetExtractor:
	"""@brief Extractor of TIC datasets based on a link-layer TIC frame
	"""
	LF = '\x0A'
	CR = '\x0D'

	def __init__(self, frame):
		"""@brief Create a TIC dataset extractor based on a raw TIC frame payload

		@param frame The raw TIC frame (without STX/ETX bytes)
		"""
		self.frame = frame

	def __iter__(self):
		while len(self.frame) > 0:
			dataset = self.get_next_dataset()
			if dataset is None:
				return
			else:
				yield dataset

	def get_next_dataset(self):
		"""@brief Continue parsing the next available dataset

		@return The new dataset as a buffer (or  None if the frame was fully decoded)
		"""
		start_dataset_pos = self.frame.find(self.LF)
		if start_dataset_pos == -1:	# No start marker, do not decode
			return None
		if start_dataset_pos != 0:
			print('Warning leading garbage in dataset')
		self.frame = self.frame[start_dataset_pos+1:]	# Remove dataset starting marker
		end_dataset_pos = self.frame.find(self.CR)
		if end_dataset_pos == -1:	# No end marker, assume we have to return the whole frame
			dataset = self.frame
			self.frame = ''
			return dataset
		else:
			dataset = self.frame[:end_dataset_pos]
			self.frame = self.frame[end_dataset_pos+1:]	# Remove the dataset end marker, and keep the trailing bytes for a next extraction
			return dataset

class TICFrames:
	"""@brief Class that allows to extract data from TIC frames
	"""
	def __init__(self, tic_link_frame_fetcher, standard_tic_mode=False):
		"""@brief Create a TIC frame parsing instance

		@param tic_link_frame_fetcher An object of type TICLinkLayerDecoder used to fetch and extract TIC datasets
		@param standard_tic_mode True if we should we decode using "TIC standard" mode, False if we should decode using "TIC historique" mode
		"""
		self._tic_link_frame_fetcher = tic_link_frame_fetcher
		self.standard_tic_mode = standard_tic_mode

	def __iter__(self):
		while True:
			yield self.get_next()

	@staticmethod
	def _checksum(data):
		chksum = sum([ord(c) for c in data])
		chksum = (chksum & 63) + 32
		return chr(chksum)

	def get_next(self):
		frame = ''
		while not frame:	# Block until a new frame is available
			frame = self._tic_link_frame_fetcher()
			if not frame:
				time.sleep(0.2)

		dataset_extractor = TICDataSetExtractor(frame=frame)
		decoded_frame = {}
		for dataset in dataset_extractor:
			if self.standard_tic_mode:
				data_payload = dataset.split("\t")
				checksummed_payload = dataset[:-1]
			else:
				data_payload = dataset.split(" ")
				checksummed_payload = dataset[:-2]

			etiquette = data_payload[0]
			if len(data_payload) == 3:
				value = data_payload[1].strip()
			elif len(data_payload) == 4: #standardMode has some fields with date before the value.
				value = (data_payload[1].strip(),data_payload[2].strip())
			else:
				print('Error: Bad data payload:' + str(dataset))
				continue

			enclosed_checksum = dataset[-1]
			computed_checksum = self._checksum(checksummed_payload)
			if enclosed_checksum == computed_checksum:
				decoded_frame[etiquette] = value
			else:
				print('Error: Bad checksum on :' + str(line) + '| checksum:' + str(chksum) + ', vs:' + str(computedchksum))

		return decoded_frame

class LinkyHorodate:
	"""@brief Class for parsing Linky TIC horodate
	"""
	def __init__(self, saison, annee, mois, jour, heure, minute, seconde):
		self.saison = saison
		self.annee = annee
		self.mois = mois
		self.jour = jour
		self.heure = heure
		self.minute = minute
		self.seconde = seconde

	def __repr__(self):
		return 'LinkyHorodate(' + 'saison=' + str(self.saison) + ',' + 'date=' + str(self.jour) + '/' + str(self.mois) + '/' + str(self.annee) + ',' + 'heure=' + str(self.heure) + ':' + str(self.minute) + ':' + str(seconde) + ')'

	@staticmethod
	def from_horodate_string(input):
		assert isinstance(input, str)
		assert len(input) == 13
		return LinkyHorodate(saison=input[0],
		                     annee=2000+int(input[1:3]),
		                     mois=int(input[3:5]),
		                     jour=int(input[5:7]),
		                     heure=int(input[7:9]),
		                     minute=int(input[9:11]),
		                     seconde=int(input[11:13]))

class FixedWidthHistoryBarGraph:
	"""@brief Database storing history of values

	The values in database can be converted to a fixed-width bar graph, for graphical display
	"""
	def __init__(self, width, history_requested_size):
		self.width = width
		if history_requested_size <= width:	# We should scale up bar graph (this is not very good-looking, but...)
			self.scale_up_factor = width // history_requested_size
			self.scale_down_factor = None
			self.history_max_size = width * self.scale_up_factor
			print('Scaling up, each sample will be displayed with ' + str(self.scale_up_factor) + ' lines')
		else:
			# We will scale down, averaging several values into one bar graph line
			self.scale_down_factor = history_requested_size // width
			if history_requested_size % width != 0: # There is a reminder, round to the next higher integer
				self.scale_down_factor += 1
			self.accumulated_values_for_scale_down = []
			self.scale_up_factor = None
			self.history_max_size = width * self.scale_down_factor
			print('Scaling down, each line will be the average of ' + str(self.scale_down_factor) + ' samples')
		self.history = []

	def append(self, value):
		if self.scale_up_factor is not None:	# Scaling up
			self.history.append(value)	# Just add the raw value
		elif self.scale_down_factor is not None:	# Scaling down
			self.accumulated_values_for_scale_down.append(value)
			if len(self.accumulated_values_for_scale_down) >= self.scale_down_factor:	# We have enough samples to perform the average and scale down
				accumulated = self.accumulated_values_for_scale_down[:self.scale_down_factor]
				self.accumulated_values_for_scale_down = []	# Empty our incoming buffer
				nb_valid_values = 0
				sum = 0
				for i in accumulated:
					if i is not None:
						nb_valid_values += 1
						sum += i
				if nb_valid_values == 0:
					value = None
				else:
					value = sum // nb_valid_values
				self.history.append(value)
		if len(self.history) > self.history_max_size:
			self.history = self.history[len(self.history)-self.history_max_size:]

	def to_fixed_width_list(self):
		if self.scale_up_factor is not None:	# Scaling up
			result = []
			for i in self.history:
				result += [i] * self.scale_up_factor
			return result
		else:	# History is already ready
			return self.history

class DisplayData:
	"""@brief Data class storing all the data to display on the LCD screen
	"""
	def __init__(self, scaled_bar_graph, displayed_power, pflow_str, read_error, beat):
		self.scaled_bar_graph = scaled_bar_graph
		self.displayed_power = displayed_power
		self.pflow_str = pflow_str
		self.read_error = read_error
		self.beat = beat

def list_scaled_to_percent(input):
	max_value = 0
	# First pass, we extract the maximum, ignoring None items
	for i in input:
		if i is not None and i>max_value:
			max_value = i
	# Second pass, we scale all values to 100%
	if max_value == 0:
		return (input, 0)
	result = []
	for i in input:
		if i is None:
			result.append(None)
		else:
			result.append((i*100)//max_value)
	return (result, max_value)

class LCDDisplay:
	"""@brief Class driving an LCD display shield
	"""
	def __init__(self, image_displayer, lcd_width, lcd_height, top, font_small, font_big):
		"""@brief Constructor

		@param image_displayer A callback function taking an Image object and displaying it to the LCD
		@param lcd_width The width of the LCD screen in pixels
		@param lcd_height The height of the LCD screen in pixels
		@param top The offset (in y coords) where we should start drawing, compared to the top of the screen
		@param font_small A font to draw tiny text strings
		@param font_big A font to draw big text strings (the withdrawn power measurement)
		"""
		if not callable(image_displayer):
			raise TypeError('image_displayer argument is not callable')
		else:
			self._image_displayer = image_displayer
		self.lcd_width = lcd_width
		self.lcd_height = lcd_height
		# Make sure to create image with mode '1' for 1-bit color (B&W display)
		self.image = Image.new('1', (self.lcd_width, self.lcd_height))
		# Get drawing object to draw on image.
		self.image_drawer = ImageDraw.Draw(self.image)
		self.top = top
		self.font_small = font_small
		self.font_big = font_big
		# Draw a white filled box to clear the image.
		self.clear()

	def clear(self):
		self.image_drawer.rectangle((0, 0, self.lcd_width, self.lcd_height),
		                            outline=255,
		                            fill=255)

	def display(self):
		self._image_displayer(self.image)

	def draw_to_image(self, display_data):
		self.clear()
		self.image_drawer.text((0, self.top+4), str(display_data.displayed_power) + 'W', font=self.font_big)
		self.image_drawer.line((0, self.top+22, self.lcd_width, self.top+22), fill=0)
		if pflow_str is not None:
			self.image_drawer.text((0, self.top+23), 'Bilan:' + display_data.pflow_str + 'W', font=self.font_small)
		if not display_data.read_error:
			self.image_drawer.text((0, self.top), "Puissance soutiree", font=self.font_small)
		else:
			self.image_drawer.text((0, self.top), "Erreur lecture", font=self.font_small)
		if display_data.beat:
			self._draw_elec_icon((75, self.top+8))
		self._draw_percentage_graph((0, self.top+31, self.lcd_width, self.lcd_height), display_data.scaled_bar_graph[-self.lcd_width:])

	def _draw_percentage_graph(self, xyxy, data):
		assert isinstance(xyxy, tuple)
		(xleft, ytop, xright, ybottom) = xyxy
		x = xleft
		y = ytop
		height = ybottom - ytop
		width = xright - xleft
		assert x == 0
		# Crop data to the max width
		data = data[:width]
		ofs = 0
		bottom_y = y+height-1
		for value in data:
			bar_sz = value * height // 100	# Scale the percentage to the height
			self.image_drawer.line((x+ofs, bottom_y-bar_sz, x+ofs, bottom_y), fill=0)
			ofs += 1

	def _draw_elec_icon(self, xy):
		assert isinstance(xy, tuple)
		(x, y) = xy
		self.image_drawer.line((x, y, x+5, y+5), fill=0)
		self.image_drawer.line((x+5, y+5, x, y+5), fill=0)
		self.image_drawer.line((x, y+5, x+5, y+10), fill=0)
		self.image_drawer.line((x+5, y+10, x+2, y+10), fill=0)
		self.image_drawer.line((x+5, y+10, x+5, y+7), fill=0)

def evaluate_power_flow(current, voltage, is_injecting):
	"""@brief Try to guess a range containing the current power flow (positive=withdrawn from the grid, negative=injected on the grid)

	@param current The instantaneous rms current
	@param voltage The instantaneous rms voltage
	@param is_injecting True if we are injecting to the grid
	"""
	pflow_min = (irms+0.5) * urms
	pflow_max = (irms-0.5) * urms
	if inject:
		(pflow_min, pflow_max) = (-pflow_min, -pflow_max)	# Negative values when injecting
	if pflow_min>pflow_max:
		(pflow_min, pflow_max) = (pflow_max, pflow_min)
	pflow_str = '[' + str(int(round(pflow_min))) + ';' + str(int(round(pflow_max))) + ']'
	return (pflow_str, pflow_min, pflow_max)

if __name__ == "__main__":
	print('Starting...')
	# Raspberry Pi software SPI config:
	SCLK = 17
	DIN = 18
	DC = 27
	RST = 23
	CS = 22

	# Software SPI usage (defaults to bit-bang SPI interface):
	disp = LCD.PCD8544(DC, RST, SCLK, DIN, CS)

	# Initialize library.
	disp.begin(contrast=60)

	# Clear display.
	disp.clear()
	disp.display()

	def image_displayer(image):
		disp.image(image)
		disp.display()

	display_driver = LCDDisplay(image_displayer=image_displayer,
	                            lcd_width=LCD.LCDWIDTH, lcd_height=LCD.LCDHEIGHT, top=-1,
	                            font_small=ImageFont.truetype('DejaVuSans.ttf', 8),
	                            font_big=ImageFont.truetype('DejaVuSans.ttf', 18))

	phy = PhyDecoder(baudrate=9600, port="/dev/ttyUSB0")
	link_decoder = TICLinkLayerDecoder(phy)
	tic_frames = TICFrames(tic_link_frame_fetcher=link_decoder.get_next_frame, standard_tic_mode=True)
	hist = FixedWidthHistoryBarGraph(width=LCD.LCDWIDTH, history_requested_size=60*15) # Collect the last 15 minutes of power measurement in the lower graph
	beat = True
	successive_sinsts_errors = 0
	last_sinsts = -1
	successive_null_sinsts = 0

	display_queue = Queue.Queue()

	def display_to_lcd(data_queue):
		while True:
			new_display_data = display_queue.get()
			while not display_queue.empty():
				print('Running late... discarding one display data')
				display_queue.get()
			display_driver.draw_to_image(display_data=new_display_data)
			display_driver.display()

	#display_thread = threading.Thread(target=display_to_lcd, args=(display_queue), daemon=True) # Only available on recent python versions
	display_thread = threading.Thread(target=display_to_lcd, args=(display_queue,))
	display_thread.start()

	for frame in tic_frames:
		# Historical code to get current CPU temp (unused)
		#with open("/sys/class/thermal/thermal_zone0/temp") as temp_f:
		#	temp = float(temp_f.read()) / 1000.0
		#temp = round(temp, 1)
		read_error = False
		power = None
		new_switch_to_withdrawn_power = False
		try:
			last_sinsts=int(frame['SINSTS'])
			power = last_sinsts
			successive_sinsts_errors = 0
			if last_sinsts == 0:
				successive_null_sinsts += 1
			else:
				if successive_null_sinsts > 10:
					new_switch_to_withdrawn_power = True
				successive_null_sinsts = 0
		except Exception as e:
			successive_sinsts_errors += 1
			read_error = True
			tb = traceback.format_exc()
			print(tb)
		# Power has an up-to-date value (and thus is not None) only if SINSTS read was successful
		hist.append(power)	# Add the current reading (or None if reading failed)
		(scaled_bar_graph, max_value) = list_scaled_to_percent(input=hist.to_fixed_width_list())
		if new_switch_to_withdrawn_power:
			prefix=TerminalColor.FAIL
		else:
			prefix=''
		print(prefix + '(MAX=' + str(max_value) + 'W)' + str(scaled_bar_graph) + TerminalColor.ENDC)
		displayed_power = power
		if successive_sinsts_errors < 3:	# We keep drawing the previous SINST value until 2 successive errors
			displayed_power = last_sinsts	# Set power to the last known power (old reading)
		if successive_sinsts_errors == 0:
			print('Power=' + str(displayed_power) + 'W')
		pflow_str = None
		irms = None
		inject = (power == 0)
		if not read_error:
			try:
				irms=int(frame['IRMS1'])
				urms=int(frame['URMS1'])
				(pflow_str, _, _) = evaluate_power_flow(current=irms, voltage=urms, is_injecting=inject)
			except Exception as e:
				read_error = True
				tb = traceback.format_exc()
				print(tb)
		if pflow_str is not None:
			if irms is not None:
				prefix=''
				if irms == 0:	# We are around 0 injection, 0 withdrawn, use yellow
					prefix=TerminalColor.WARNING
				elif inject:
					prefix=TerminalColor.OKGREEN
				else:
					prefix=TerminalColor.FAIL
			print(prefix + 'Pflow=' + pflow_str + 'W' + TerminalColor.ENDC)


		new_display_data = DisplayData(scaled_bar_graph=scaled_bar_graph, displayed_power=displayed_power, pflow_str=pflow_str, read_error=read_error, beat=beat)
		display_queue.put_nowait(new_display_data)

		if successive_sinsts_errors == 0:
			beat = not beat
