#!/usr/bin/env python

##@file wav2c.py
#  @ingroup util
#	A script for converting wav 8/16 bit sound data files to wavetables for c applications.
#
#	Usage: 
#	>>>wav2c.py <infile outfile tablename samplerate>
#	
#	@param infile		The file to convert, 8/16 bit PCM.  wav file
#	@param outfile	    The file to save as output, a .h file containing a table .
#	@param tablename	The name to give the table of converted data in the new file.
#	@param samplerate	The samplerate the sound was recorded at.  Choose what make sense for you, if it's not a normal recorded sample.
#
#	@note Batch wav conversion
#   1. first sox : for file in *.WAV; do sox $file  -r 10000 -D -c 1 -b 8  "$(basename $file .wav).wav" -V; done 
#      or with trim "$(basename $file .wav).wav" trim 0 -0.5; done
#   2. now headers : 
#      for file in *.WAV; do ../wav2c.py "$(basename ${file%})" ${file%.*}.h "${file%.*}" 11000 ; done
#
#	@note Using Audacity to prepare sound files for converting:

#	For generated waveforms like sine or sawtooth, set the project
#	rate to the size of the wavetable you wish to create, which must
#	be a power of two (eg. 8192), and set the selection format
#	(beneath the editing window) to samples. Then you can generate
#	and save 1 second of a waveform and it will fit your table
#	length.
#	
#	For a recorded audio sample, set the project rate to the
#	AUDIO_RATE (16384 in the mozzi current version). 
#	Samples can be any length, as long as they fit in your Arduino.
#	
#	Save by exporting with the format set to "wav",
#	"Encoding: Unsigned 8 bit PCM".
#	
#	Now use the file you just exported, as the "infile" to convert.
#
#	@author Tim Barrass 2010-12 Mark Washeim 2025
#	@fn wav2c

import sys, array, os, textwrap, random, wave

if len(sys.argv) != 5:
        print ('Usage: wav2c.py <infile outfile tablename samplerate>')
        sys.exit(1)

[infile, outfile, tablename, samplerate] = sys.argv[1:]

def wav2c(infile, outfile, tablename, samplerate):
    with wave.open(infile) as wavf:
        meta = wavf.getparams()
        frames = wavf.readframes(meta.nframes)

    print ("opened " + infile)

    pcm_samples = array.array("B", frames) # B is unsigned int8
    length = len(pcm_samples)

    values=pcm_samples.tolist()

    fout = open(os.path.expanduser(outfile), "w")
    fout.write('#ifndef ' + tablename + '_H_' + '\n')
    fout.write('#define ' + tablename + '_H_' + '\n \n')
    fout.write('const uint16_t ' + tablename + 'Len = ' + str(length) + ';\n\n')
    outstring = 'const uint8_t ' + tablename + '[' + str(length) + '] PROGMEM = {'

    try:
        for i in range(len(values)):
        ## mega2560 boards won't upload if there is 33, 33, 33 in the array, so dither the 3rd 33 if there is one
            if (values[i] == 33) and (values[i+1] == 33) and (values[i+2] == 33):
                values[i+2] = random.choice([32, 34])
            outstring += str(values[i]) + ", "
    finally:
        outstring +=  "};"
        outstring = textwrap.fill(outstring, 80)
        fout.write(outstring)
        fout.write('\n\n#endif /* ' + tablename + '_H_ */\n')
        fout.close()
    print ("wrote " + outfile)

wav2c(infile, outfile, tablename, samplerate)
