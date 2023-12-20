import numpy as np


"""
	Stuffing function. Will mostly be unused, since it's only really good for PC to PC communication (microcontrollers have IDLE line detection)
	or Arduinos via. serial
	
	Inputs: input array. Must be a bytearray type object! can create the following two ways:
		example 1: input_barr = bytearray([1,2,3])
		example 2: input_barr = np.uint8([1,2,3]).tobytes()
	numpy supports ".tobytes()" which converts to bytearray type
	
	
	returns a PPP stuffed bytearray
"""
def PPP_stuff(input_barr):
	FRAME_CHAR = np.uint8(0x7E)
	ESC_CHAR = np.uint8(0x7D)
	ESC_MASK = np.uint8(0x20)

	#to start, convert to np array for array ops
	working_buf = np.frombuffer(input_barr, dtype=np.uint8).copy()		#copy bc i think this means otherwise we're writing to the original array, but i want this to be a copy


	#first logical op, find all instances of the ESC char, prepend the escape char, and xor them with 0x20
	inds = np.where(working_buf==ESC_CHAR)[0]
	working_buf[inds] = np.bitwise_xor(working_buf[inds],ESC_MASK)
	working_buf = np.insert(working_buf, inds, ESC_CHAR)

	
	#second, find all frame chars in data, prepend the escape char, and xor them with 0x20
	inds = np.where(working_buf==FRAME_CHAR)[0]
	working_buf[inds] = np.bitwise_xor(working_buf[inds],ESC_MASK)
	working_buf = np.insert(working_buf, inds, ESC_CHAR)

	#finally, prepend and postpend the frame characters 
	working_buf = np.insert(working_buf, 0, FRAME_CHAR)	
	working_buf = np.append(working_buf, FRAME_CHAR)
		
		
	b = working_buf.tobytes()
	return b
	
"""
	Unstuff operation, which is basically a helper function for unstuff_PPP_stream
	
	also must be a bytearray type object. see above comment and test.py for more information
"""
def PPP_unstuff(input_barr):
	FRAME_CHAR = np.uint8(0x7E)
	ESC_CHAR = np.uint8(0x7D)
	ESC_MASK = np.uint8(0x20)

	wip = np.frombuffer(input_barr, dtype = np.uint8)
	working_input = wip.copy()
	
	
	if(working_input[0] != FRAME_CHAR or working_input[working_input.size-1] != FRAME_CHAR):
		return np.array([])
	
	
	inds = (np.where(working_input==ESC_CHAR)[0] + 1)	#locate all bytes which directly follow an escape character
	working_input[inds] = np.bitwise_xor(working_input[inds], ESC_MASK)	#xor the
	working_input = np.delete(working_input, inds-1)
	
	b = working_input[1:(working_input.size-1)].tobytes()
	return b
	
	
"""
	Unstuffing, but it queues new bytes as they come in and performs framing logic on the stream.
	Creates a local buffer starting with the first frame character found, and deletes it to restart every time there's a new frame character
	
	
	In implementation, combine this with a CRC or checksum for data integrity confirmation. Alignment errors/dropped or malformed bytes can cause frames to pass which are malformed, and they should be rejected in those cases
	
	Must be called on input data ONE BYTE AT A TIME.
	That means if you have an array, you gotta wrap this in a for loop.
	
	Hopefully speed isn't a problem... cuz it'll def break if so
"""
def unstuff_PPP_stream(new_byte, stuff_buffer):
	FRAME_CHAR = np.uint8(0x7E)
	
	stuff_buffer = np.append(stuff_buffer, np.uint8(new_byte))
	payload = np.array([]).tobytes()
	if(new_byte == FRAME_CHAR):
		payload = PPP_unstuff(stuff_buffer.tobytes())
		stuff_buffer = np.array([np.uint8(new_byte)])	#reset stuff buffer size and cram the first element with the frame character
		
	return payload, stuff_buffer
	
