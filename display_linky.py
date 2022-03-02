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
import display_linky
s=display_linky.PhyDecoder(baudrate=9600, port="/dev/ttyUSB0")
l=display_linky.TICLinkLayer(s)
tf=display_linky.TICFrame(l,standard_tic_mode=True)
for frame in tf:
 print(frame)

"""

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
		self.frame = frame
	
	def __iter__(self):
		while len(self.frame) > 0:
			dataset = self.get_next_dataset()
			if dataset is None:
				return
			else:
				yield dataset
	
	def get_next_dataset(self):
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
	"""@brief Class representing all the data displayed on the LCD screen
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

def draw_elec_icon(image_draw, xy):
	assert isinstance(xy, tuple)
	(x, y) = xy
	image_draw.line((x, y, x+5, y+5), fill=0)
	image_draw.line((x+5, y+5, x, y+5), fill=0)
	image_draw.line((x, y+5, x+5, y+10), fill=0)
	image_draw.line((x+5, y+10, x+2, y+10), fill=0)
	image_draw.line((x+5, y+10, x+5, y+7), fill=0)

def draw_percentage_graph(image_draw, xyxy, data):
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
	for value in data:
		bar_sz = value * height // 100	# Scale the percentage to the height
		image_draw.line((x+ofs, y+height-bar_sz, x+ofs, y+height), fill=0)
		ofs += 1

def draw_to_lcd(image_draw, top, lcd_width, lcd_height, font_big, font_small, display_data):
	image_draw.rectangle((0,0,lcd_width,lcd_height), outline=255, fill=255)
	draw_percentage_graph(image_draw, (0, top+31, lcd_width, lcd_height), display_data.scaled_bar_graph)
	image_draw.text((0, top+4), str(display_data.displayed_power) + 'W', font=font_big)
	image_draw.line((0, top+22, lcd_width, top+22), fill=0)
	if pflow_str is not None:
		image_draw.text((0, top+23), 'Bilan:' + display_data.pflow_str + 'W', font=font_small)
	if not display_data.read_error:
		image_draw.text((0, top), "Puissance soutiree", font=font_small)
	else:
		image_draw.text((0, top), "Erreur lecture", font=font_small)
	if display_data.beat:
		draw_elec_icon(image_draw, (75, top+8))

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

	# Create blank image for drawing.
	# Make sure to create image with mode '1' for 1-bit color.
	width = LCD.LCDWIDTH
	height = LCD.LCDHEIGHT
	image = Image.new('1', (width, height))

	# Get drawing object to draw on image.
	draw = ImageDraw.Draw(image)

	# Draw a white filled box to clear the image.
	draw.rectangle((0,0,LCD.LCDWIDTH,LCD.LCDHEIGHT), outline=255, fill=255)

	# Load default font.
	#font = ImageFont.load_default()
	font_small=ImageFont.truetype('DejaVuSans.ttf', 8)
	font_big=ImageFont.truetype('DejaVuSans.ttf', 18)

	top=-1

	phy = PhyDecoder(baudrate=9600, port="/dev/ttyUSB0")
	link_decoder = TICLinkLayerDecoder(phy)
	tic_frames = TICFrames(tic_link_frame_fetcher=link_decoder.get_next_frame, standard_tic_mode=True)
	hist = FixedWidthHistoryBarGraph(width=LCD.LCDWIDTH, history_requested_size=60*15) # Collect the last 15 minutes of power measurement in the lower graph
	beat = True
	successive_sinsts_errors = 0
	last_sinsts = -1

	display_queue = Queue.Queue()

	def display_to_lcd(data_queue):
		print('Display thread start')
		while True:
			new_display_data = display_queue.get()
			while not display_queue.empty():
				print('Running late... discarding one display data')
				display_queue.get()
			print('Drawing')
			draw_to_lcd(image_draw=draw,
				    top=top, lcd_width=LCD.LCDWIDTH, lcd_height=LCD.LCDHEIGHT,
				    font_big=font_big, font_small=font_small,
				    display_data=new_display_data)
			print('Starting xfer to LCD display')
			disp.image(image)
			disp.display()
			print('xfer to LCD display done')

	#display_thread = threading.Thread(target=display_to_lcd, args=(display_queue), daemon=True)
	display_thread = threading.Thread(target=display_to_lcd, args=(display_queue,))
	display_thread.start()

	for frame in tic_frames:
		''' cputemp '''
		#with open("/sys/class/thermal/thermal_zone0/temp") as temp_f:
		#	temp = float(temp_f.read()) / 1000.0
		#temp = round(temp, 1)
		read_error = False
		power = None
		try:
			last_sinsts=int(frame['SINSTS'])
			power = last_sinsts
			successive_sinsts_errors = 0
		except Exception as e:
			successive_sinsts_errors += 1
			read_error = True
			tb = traceback.format_exc()
			print(tb)
		# Power has an up-to-date value (and thus is not None) only if SINSTS read was successful
		hist.append(power)	# Add the current reading (or None if reading failed)
		(scaled_bar_graph, max_value) = list_scaled_to_percent(input=hist.to_fixed_width_list())
		print(str(scaled_bar_graph))
		displayed_power = power
		if successive_sinsts_errors < 3:	# We keep drawing the previous SINST value until 2 successive errors
			displayed_power = last_sinsts	# Set power to the last known power (old reading)
		if successive_sinsts_errors == 0:
			print('Power=' + str(displayed_power) + 'W')
		pflow_str = None
		if not read_error:
			try:
				irms=int(frame['IRMS1'])
				urms=int(frame['URMS1'])
				inject = (power == 0)
				if True:
					# We are injecting
					pflow_min = (irms+0.5) * urms
					pflow_max = (irms-0.5) * urms
					if inject:
						(pflow_min, pflow_max) = (-pflow_min, -pflow_max)	# Negative values when injecting
					if pflow_min>pflow_max:
						(pflow_min, pflow_max) = (pflow_max, pflow_min)
					pflow_str = '[' + str(int(round(pflow_min))) + ';' + str(int(round(pflow_max))) + ']'
				else:
					pflow_str = str(power)
			except Exception as e:
				read_error = True
				tb = traceback.format_exc()
				print(tb)
		if pflow_str is not None:
			print('Pflow=' + pflow_str + 'W')
		
		new_display_data = DisplayData(scaled_bar_graph=scaled_bar_graph, displayed_power=displayed_power, pflow_str=pflow_str, read_error=read_error, beat=beat)
		display_queue.put_nowait(new_display_data)
		
		if successive_sinsts_errors == 0:
			beat = not beat
