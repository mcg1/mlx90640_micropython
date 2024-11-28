## Reworking of the Adafruit MLX90640 CPP driver for micropython.
## See github.com/adafruit/Adafruit_MLX90640 for original CPP source driver 


from machine import I2C
from micropython import const
from micropython import mem_info
from array import array
import math
import struct


#I2C functions for the MLX sensor
class MLX90640_I2C:    
    #Default address for the sensor
    __SENSOR_ADDRESS = 0x33
    __bufferSize = 1024
    __sendBuffer = bytearray(__bufferSize)    
    __recBuffer = bytearray(__bufferSize)

    def __init__(self, i2c: I2C) -> None:
        self._i2c = i2c

    @micropython.native # type: ignore
    def ReadRegister(self, register:int) ->int:        
        sendBuffer=self.__sendBuffer
        register &= 65535
        sendBuffer[0] = register >> 8
        sendBuffer[1] = register & 0xff        
        mv = memoryview(self.__recBuffer)
        smv = memoryview(self.__sendBuffer)        
        i2c = self._i2c
        addr = self.__SENSOR_ADDRESS
        i2c.writeto(addr, smv[0:2], False)            
        i2c.readfrom_into(addr,mv[0:2])
        value = (mv[0]<<8) | (mv[1])
        return value
    
    @micropython.native # type: ignore
    def WriteRegister(self, register:int, value:int) ->int:        
        sendBuffer=self.__sendBuffer
        register &= 65535
        sendBuffer[0] = register >> 8
        sendBuffer[1] = register & 0xff
        sendBuffer[2] = (value & 0xff00) >> 8
        sendBuffer[3] = value & 0xff
        smv = memoryview(self.__sendBuffer)
        i2c = self._i2c
        addr = self.__SENSOR_ADDRESS
        i2c.writeto(addr, smv[0:4], True)        
        return value
    
    @micropython.native # type: ignore
    def ReadRegisters(self, register:int, count:int) -> array:
        result = array('H', bytearray(2*count))                
        maxlen = self.__bufferSize
        sendBuffer=self.__sendBuffer
        i2c = self._i2c
        addr = self.__SENSOR_ADDRESS
        rb = memoryview(self.__recBuffer)
        smv = memoryview(self.__sendBuffer)[0:2]
        offset = 0 

        while count>0:
            readlen = count * 2 #2 bytes per register            
            if readlen > maxlen:
                readlen = maxlen
            readShorts = readlen >> 1
            sendBuffer[0] = register >> 8
            sendBuffer[1] = register & 0xff
            i2c.writeto(addr, smv, False)            
            i2c.readfrom_into(addr,rb[0:readlen])
            for i in range(readShorts):
                ri = 2*i
                result[offset + i] = (rb[ri]<<8) | (rb[ri+1])            
            offset += readShorts
            count -= readShorts
            register += readShorts

        return result
    
    @micropython.native # type: ignore
    def ReadRegistersInto(self, register:int, count:int, buffer) -> None:                      
        maxlen = self.__bufferSize
        sendBuffer=self.__sendBuffer
        i2c = self._i2c
        addr = self.__SENSOR_ADDRESS
        rb = memoryview(self.__recBuffer)
        smv = memoryview(self.__sendBuffer)[0:2]
        offset = 0 

        while count>0:
            readlen = count * 2 #2 bytes per register            
            if readlen > maxlen:
                readlen = maxlen
            readShorts = readlen >> 1
            sendBuffer[0] = register >> 8
            sendBuffer[1] = register & 0xff
            i2c.writeto(addr, smv, False)            
            i2c.readfrom_into(addr,rb[0:readlen])            
            for i in range(readShorts):
                ri = 2*i
                buffer[offset + i] = (rb[ri]<<8) | (rb[ri+1])            
            offset += readShorts
            count -= readShorts
            register += readShorts        
    
    @micropython.native # type: ignore
    def is_Sensor_Connected(self) -> bool:
        #Try to locate the device on the bus        
        found_devices = self._i2c.scan()
        if not self.__SENSOR_ADDRESS in found_devices:
            print("Device not found at i2c address ", self.__SENSOR_ADDRESS)
            return False            
        ee_regVal = self.ReadRegister(0x240f) #Grab the saved I2C address from the EEPROM register
        ee_regVal &= 0xff #Stored address is in the lower byte
        if ee_regVal != self.__SENSOR_ADDRESS: #It should match the address we are using - simple device verification
            print("Unexpected result from device at ", self.__SENSOR_ADDRESS)
            return False        
        return True    


class MLX90640:        
    #Class Init
    __MODULE_NAME="MLX90640"
    __EEPROM_ADDR = 0x2400
    __EEPROM_SIZE = 832
    __SCALE_ALPHA = 0.000001
    __OPENAIR_TA_SHIFT = 8

    RESOLUTION_16_BIT = 0
    RESOLUTION_17_BIT = 1
    RESOLUTION_18_BIT = 2
    RESOLUTION_19_BIT = 3

    REFRESH_0_HZ = 0
    REFRESH_0_5_HZ = 1
    REFRESH_1_HZ = 2
    REFRESH_2_HZ = 3
    REFRESH_4_HZ = 4
    REFRESH_8_HZ = 5
    REFRESH_16_HZ = 6
    REFRESH_32_HZ = 7

    def __init__(self, i2c: I2C) -> None:
        self._i2c:MLX90640_I2C = MLX90640_I2C(i2c)
        self.is_connected = self._i2c.is_Sensor_Connected()
        self.__print_message(f"Device Connected = {self.is_connected}")
        eeprom = self.__dump_EEPROM()
        self.__unpack_vdd_parameters(eeprom)
        self.__unpack_ptat_parameters(eeprom)        
        self.__unpack_gain_parameters(eeprom)
        self.__unpack_tgc_parameters(eeprom)
        self.__unpack_adc_resolution_parameters(eeprom)
        self.__unpack_ksto_parameters(eeprom)
        self.__unpack_ksta_parameters(eeprom)
        self.__unpack_cpp_parameters(eeprom)
        self.__unpack_alpha_parameters(eeprom)
        self.__unpack_offset_parameters(eeprom)
        self.__unpack_kta_pixel_calibration(eeprom)
        self.__unpack_kv_pixel_calibration(eeprom)
        self.__unpack_cil_parameters(eeprom)

        # some pre-calc
        # In the Adafruit library, this was in the Calculateo function
        self.__cal_alphaCorrR[0] = 1 / (1+self.__cal_ksTo[0] * 40)
        self.__cal_alphaCorrR[1] = 1
        self.__cal_alphaCorrR[2] = (1 + self.__cal_ksTo[1] * self.__cal_ct[2])
        self.__cal_alphaCorrR[3] = self.__cal_alphaCorrR[2] * (1 + self.__cal_ksTo[2] * (self.__cal_ct[3] - self.__cal_ct[2]))
        
        self._i2c.is_Sensor_Connected()

        self.set_chess_mode()           
        #self.set_interleaved_mode()
        self.set_adc_resolution(self.RESOLUTION_18_BIT)
        self.set_refresh_rate(self.REFRESH_8_HZ)
        
        print (self.get_adc_resolution())
        print (mem_info())

        print (self.__cal_kv)
        print (self.__cal_kta)


    def get_frame_array(self):
        return array('f', bytearray(4*768))
    

    __frame_readbuffer1:array = array('H', bytearray(2 * 834)) #int16 x 834
    __frame_readbuffer2:array = array('H', bytearray(2 * 834)) #int16 x 834
    __cal_alphaCorrR:array = array('f', bytearray(4 * 4)) #float x 4

    @micropython.native # type: ignore
    def read_frame_into(self, frame_array:array) -> bool:
        emissivity = 1.0# 0.95
        
        #Processing takes too long so have to read both pages in first and then process them after
        frame_readbuffer1 = self.__frame_readbuffer1
        frame_readbuffer2 = self.__frame_readbuffer2
        self.__wait_read_frame_data(frame_readbuffer1)
        self.__wait_read_frame_data(frame_readbuffer2)
        if frame_readbuffer1[833] == frame_readbuffer2[833]:
            print ("Read same page")
        t_amb = self.__get_tamb_from_frame_data(frame_readbuffer1)
        t_reflected = t_amb - self.__OPENAIR_TA_SHIFT  #Use the corrected ambient temperature for reflectance
        
        ##TODO Merge the two loops into one and just pick the right buffer for the pixel
        self.__calculate_pixel_temperatures(frame_readbuffer1, frame_array, emissivity, t_reflected)
        self.__calculate_pixel_temperatures(frame_readbuffer2, frame_array, emissivity, t_reflected)
        #for i in range(2):            
#            self.__wait_read_frame_data(frame_readbuffer)
            #t_amb = self.__get_tamb_from_frame_data(frame_readbuffer)
            #t_reflected = t_amb - self.__OPENAIR_TA_SHIFT  #Use the corrected ambient temperature for reflectance
            #self.__calculate_pixel_temperatures(frame_readbuffer, frame_array, emissivity, t_reflected)

        print(t_amb)
        print(frame_array[767])
        return False

    @micropython.native # type: ignore
    def __calculate_pixel_temperatures(self, frame_read_buffer, output_buffer, emissivity, t_reflected):
        subpage = frame_read_buffer[833]
        vdd = self.__get_vdd_from_frame_data(frame_read_buffer)
        ta = self.__get_tamb_from_frame_data(frame_read_buffer)

        ta4 = (ta + 273.15)
        ta4 *= ta4
        ta4 *= ta4
        tr4 = (t_reflected + 273.15)
        tr4 *= tr4
        tr4 *= tr4
        taTr = tr4 - (tr4-ta4)/emissivity

        ta -= 25 # from this point on ta is never used without subtracting 25, so just do it once here
        vdd -= 3.3 # same with vdd

        ktaScale = float(1<<self.__cal_kta_scale)
        kvScale = float(1<< self.__cal_kv_scale)
        alphaScale = float(1<< self.__cal_alphaScale)
        alphaCorrR = self.__cal_alphaCorrR

        gain = frame_read_buffer[778]
        if gain > 32767: gain -= 65536
        gain = self.__cal_gainEE / gain

        mode:int = (frame_read_buffer[832] & 0x1000)>>5

        irDataCP = [frame_read_buffer[776], frame_read_buffer[808]]
        if irDataCP[0] > 32767 : irDataCP[0] -= 65536
        if irDataCP[1] > 32767 : irDataCP[1] -= 65536
        irDataCP[0] *= gain
        irDataCP[1] *= gain

        ktakv_premul:float = (1 + self.__cal_cpKta * ta) * (1+self.__cal_cpKv * vdd)

        irDataCP[0] = irDataCP[0] - self.__cal_cpOffset[0] * ktakv_premul
        if mode == self.__cal_cal_mode_ee:
            irDataCP[1] = irDataCP[1] - self.__cal_cpOffset[1] * ktakv_premul
        else:
            irDataCP[1] = irDataCP[1] - (self.__cal_cpOffset[1] + self.__cal_ilChessC[0])  * ktakv_premul

        patternMode = frame_read_buffer[833]        

        cal_kta = self.__cal_kta
        cal_ksto = self.__cal_ksTo
        ksto_precalc = 1-cal_ksto[1] * 273.15
        cal_kv = self.__cal_kv
        cal_offset = self.__cal_offset
        cal_cal_mode_ee = self.__cal_cal_mode_ee
        cal_ilChessC = self.__cal_ilChessC
        cal_tgc = self.__cal_tgc    
        cal_alpha = self.__cal_alpha
        ksTa_precalc = (1 + self.__cal_ksTa * ta)
        cal_ct = self.__cal_ct
        
        irDataCP_subpage = irDataCP[subpage]

        alphaScale *= self.__SCALE_ALPHA
        print (subpage)
        for pixelNum in range(768):
            ilPattern = int((pixelNum>>5) - (pixelNum>>6)*2)
            chessPattern = int(ilPattern ^ (pixelNum - (pixelNum>>1)*2))        

            #ilPattern = int(pixelNum / 32) - int(pixelNum / 64) * 2 
            #chessPattern = ilPattern ^ (pixelNum - int(pixelNum&0xfffe))
            pattern = ilPattern if mode == 0 else chessPattern
            
            if pattern != patternMode:
                continue
            
            irData = frame_read_buffer[pixelNum]
            if irData > 32767: irData -= 65536
            irData *= gain
            ##TODO - Prescale these values
            kta:float = float(cal_kta[pixelNum]) / ktaScale
            kv:float = float(cal_kv[pixelNum]) / kvScale

            irData = irData - cal_offset[pixelNum] * (1+kta*ta) * (1+kv*vdd)
            
            if mode != cal_cal_mode_ee:                
                convPattern = int(((pixelNum +2) /4 - (pixelNum +3) /4 + (pixelNum +1) /4 - pixelNum/4) * (1-2*ilPattern))
                irData = irData + cal_ilChessC[2] * (2*ilPattern -1) - cal_ilChessC[1] * convPattern
            
            irData = irData - cal_tgc * irDataCP_subpage            
            irData /= emissivity

            
            alphaComp = alphaScale / cal_alpha[pixelNum]
            alphaComp = alphaComp * ksTa_precalc

            sx = alphaComp * alphaComp * alphaComp * (irData + alphaComp * taTr)
            sx = math.sqrt(math.sqrt(sx)) * cal_ksto[1]

            to = math.sqrt(math.sqrt(irData/(alphaComp * ksto_precalc + sx) + taTr)) - 273.15
            #output_buffer[pixelNum] = to
            
            rng = 3
            if to < cal_ct[0]:
                rng = 0
            elif to < cal_ct[1]:
                rng = 1
            elif to < cal_ct[2]:
                rng = 2
            else:
                rng = 3

            #print (rng)
            
            to = math.sqrt(math.sqrt(irData / (alphaComp * alphaCorrR[rng] * (1+cal_ksto[rng] * (to - cal_ct[rng]))) + taTr)) - 273.15

            output_buffer[pixelNum] = to


    @micropython.native # type: ignore
    def __wait_read_frame_data(self, read_buffer) -> bool:
        dataReady = 0
        i2c = self._i2c
        while(dataReady == 0):
            dataReady = i2c.ReadRegister(0x8000) & 0x0008
        loopCount = 0
        statusreg = 1
        while (dataReady !=0) & (loopCount < 5):
            i2c.WriteRegister(0x8000, 0x0030)
            i2c.ReadRegistersInto(0x0400,832,read_buffer)
            statusreg = i2c.ReadRegister(0x8000)
            dataReady =  statusreg & 0x0008
            loopCount += 1
        if loopCount > 4:
            print ("Too many retry attempts")
            return False
        #Tag on the control and status registers
        cr1 = i2c.ReadRegister(0x800D)
        read_buffer[832] = cr1
        read_buffer[833] = statusreg & 0x1
        return True

    @micropython.native # type: ignore
    def __get_tamb_from_frame_data(self, frame_data) -> float:
        vdd = self.__get_vdd_from_frame_data(frame_data)
        ptat:float = frame_data[800]
        if ptat > 32767: ptat -= 65536
        ptatArt:float = frame_data[768]
        if ptatArt > 32767: ptatArt -= 65536
        ptatArt = (ptat / (ptat * self.__cal_alphaPTAT + ptatArt)) * (1<<18)
        ta = (ptatArt / (1 + self.__cal_KvPTAT * (vdd - 3.3)) - self.__cal_vPTAT25)
        ta /= self.__cal_KtPTAT
        ta += 25
        return ta
        
    @micropython.native # type: ignore
    def __get_vdd_from_frame_data(self, frame_data) -> float:
        vdd = frame_data[810]
        if vdd > 32767:vdd -= 65536
        ramres = (frame_data[832]&0x0c00)>>10
        rescor = (1<<self.__cal_resolutionEE) / (1<<ramres)
        vdd = (rescor * vdd - self.__cal_vdd25) / self.__cal_kVdd + 3.3
        return vdd



    @staticmethod 
    def get_width_pixels():
        return 32
    
    @staticmethod
    def get_height_pixels():
        return 24
    
    def set_refresh_rate(self, refresh:int):
        value = self._i2c.ReadRegister(0x800D)
        n = (refresh & 0x7) << 7
        value = (value & 0xFC7F) | n
        self._i2c.WriteRegister(0x800D,value)
        self.__print_message(f"Set to refresh mode to {refresh}")

    def set_adc_resolution(self, resolution):
        value = self._i2c.ReadRegister(0x800D)
        res = (resolution & 0x3)<<10
        value = (value & 0xf3ff) | res
        self._i2c.WriteRegister(0x800D,value)
        self.__print_message(f"Set to resolution mode to {resolution}")

    def get_adc_resolution(self) -> int:
        value = self._i2c.ReadRegister(0x800D)
        value = (value & 0x0c00)>>10
        return value
        

    def set_chess_mode(self):
        value = self._i2c.ReadRegister(0x800D)        
        value = value | 0x1000        
        self._i2c.WriteRegister(0x800D, value)
        self.__print_message("Set to chess readout mode")

    def set_interleaved_mode(self):
        value = self._i2c.ReadRegister(0x800D)        
        value = value & 0xefff        
        self._i2c.WriteRegister(0x800D, value)        
        self.__print_message("Set to interleaved readout mode")
    

    def __print_message(self, message:str):
        print(f"[{self.__MODULE_NAME}]: {message}")


    def __dump_EEPROM(self) -> array:
        self.__print_message(f"Reading EEPROM")
        return self._i2c.ReadRegisters(self.__EEPROM_ADDR, self.__EEPROM_SIZE)
    

    def __unpack_vdd_parameters(self, eeprom:array):
        self.__print_message("Unpacking VDD calibration")
        kVdd: int = (eeprom[51]&0xff00) >> 8
        if kVdd > 127: kVdd = kVdd - 256
        kVdd *= 32
        vdd25: int = eeprom[51] & 0xff
        vdd25 = ((vdd25 - 256)<<5)-8192
        self.__cal_kVdd :int = kVdd
        self.__cal_vdd25 :int = vdd25


    def __unpack_ptat_parameters(self, eeprom:array):
        self.__print_message("Unpacking PTAT calibration")
        t:int = ((eeprom[50]&0xfc00)>>10)
        if t>31: t = t-64
        self.__cal_KvPTAT:float = float(t) / 4096.0
        
        t:int = ((eeprom[50]&0x03ff))
        if t>511: t = t-1024
        self.__cal_KtPTAT:float = float(t) / 8

        self.__cal_vPTAT25:int = eeprom[49]

        t:int = eeprom[16] & 0xf000
        self.__cal_alphaPTAT:float = float(t) / pow(2,14) + 8.0


    def __unpack_gain_parameters(self, eeprom:array):
        self.__print_message("Unpacking Gain calibration")
        t:int = eeprom[48]
        if t>32767:t=t-65536
        self.__cal_gainEE:int = t


    def __unpack_tgc_parameters(self, eeprom:array):
        self.__print_message("Unpacking TGC calibration")
        t:int = eeprom[60] & 0xff
        if t>127:t=t-256
        self.__cal_tgc = float(t)/32.0


    def __unpack_adc_resolution_parameters(self, eeprom:array):
        self.__print_message("Unpacking ADC Resolution calibration")
        t:int = (eeprom[56] & 0x3000)>>12
        self.__cal_resolutionEE:int = t


    def __unpack_ksta_parameters(self, eeprom:array):
        self.__print_message("Unpacking KaTa calibration")
        t:int = (eeprom[60]&0xff00) >> 8
        if t>127: t=t-256
        self.__cal_ksTa = float(t) / 8192.0

        
    __cal_ct:array = array('h', bytearray(2 * 5)) #ct is int16 (signed) - 5 in length
    __cal_ksTo:array = array('f', bytearray(4 * 5)) #ksTo is float x 5

    def __unpack_ksto_parameters(self, eeprom:array):
        self.__print_message("Unpacking KsTo calibration")
        step:int = (eeprom[63] & 0x3000)>>12
        step *= 10
        self.__cal_ct[0] = -40
        self.__cal_ct[1] = 0
        self.__cal_ct[2] = (eeprom[63]&0xf0)>>4
        self.__cal_ct[3] = (eeprom[63]&0xf00)>>8
        self.__cal_ct[2] = self.__cal_ct[2] * step
        self.__cal_ct[3] = self.__cal_ct[2] + self.__cal_ct[3] * step
        self.__cal_ct[4] = 400        

        print (self.__cal_ct)

        KsToScale:int = (eeprom[63] & 0xf)+8
        KsToScale = 1<<KsToScale        
        tempArray =  array('l', bytearray(4*5))
        tempArray[0] = eeprom[61] & 0xff
        tempArray[1] = (eeprom[61] & 0xff00) >> 8
        tempArray[2] = eeprom[62] & 0xff
        tempArray[3] = (eeprom[62] & 0xff00) >> 8
        for i in range(4):
            v=tempArray[i]
            if v > 127: v = v - 256
            self.__cal_ksTo[i] = float(v) / KsToScale            
        self.__cal_ksTo[4] = -0.0002 #for some reason the Adafruit driver uses an array of 5 elements but only 4 are used on the MLX datasheet


    __cal_cpOffset:array = array('h', bytearray(2 * 2)) #int16 x 2
    __cal_cpAlpha:array = array('f', bytearray(4 * 2)) #float x 2


    def __unpack_cpp_parameters(self, eeprom:array):
        self.__print_message("Unpacking CPP calibration")
        alphaScale:int = 27+((eeprom[32] & 0xf000)>>12)
        alphaScale = 1<<alphaScale

        t:int = eeprom[58] & 0x3ff
        if t>511:t=t-1024
        self.__cal_cpOffset[0] = t

        t:int = (eeprom[58] & 0xfc00)>>10
        if t>31: t=t-64
        self.__cal_cpOffset[1] = self.__cal_cpOffset[0] + t

        t:int = eeprom[57] & 0x3ff
        if t>511:t=t-1024
        self.__cal_cpAlpha[0] = float(t) / alphaScale

        t:int = (eeprom[57] & 0xfc00) >> 10
        if t>31: t=t-64
        self.__cal_cpAlpha[1] = (1+float(t)/128) * self.__cal_cpAlpha[0]

        ktaScale1:int = ((eeprom[56] & 0xf0)>>4)+8
        ktaScale1 = 1 << ktaScale1
        t:int = eeprom[59] & 0xff
        if t>127: t=t-256
        self.__cal_cpKta:float = float(t) / ktaScale1

        kvScale:int = ((eeprom[56]&0xf00)>>8)
        kvScale = 1<<kvScale
        t:int = (eeprom[59] & 0xff00) >> 8
        if t>127: t=t-256
        self.__cal_cpKv:float = float(t) / kvScale

    
    __cal_alpha:array = array('H', bytearray(2 * 768)) #uint16 x 768

    def __unpack_alpha_parameters(self, eeprom:array):
        self.__print_message("Unpacking pixel Alpha values")
        accRow = [0] * 24
        accColumn = [0] * 32
        p:int = 0
        
        accRemScale:int = eeprom[32] & 0x00f
        accColumnScale:int = (eeprom[32] & 0xf0) >> 4
        accRowScale:int = (eeprom[32] & 0xf00) >> 8
        alphaScale:int = ((eeprom[32] & 0xf000) >> 12) + 30
        alphaRef:int = eeprom[33]

        for i in range(6):
            p = i*4
            accRow[p + 0] = (eeprom[34+i] & 0x000f)
            accRow[p + 1] = (eeprom[34+i] & 0x00f0) >> 4
            accRow[p + 2] = (eeprom[34+i] & 0x0f00) >> 8
            accRow[p + 3] = (eeprom[34+i] & 0xf000) >> 12

        for i in range(24):
            if accRow[i]>7 :
                accRow[i] = accRow[i] - 16

        for i in range(8):
            p = i*4
            accColumn[p + 0] = (eeprom[40+i] & 0x000f)
            accColumn[p + 1] = (eeprom[40+i] & 0x00f0) >> 4
            accColumn[p + 2] = (eeprom[40+i] & 0x0f00) >> 8
            accColumn[p + 3] = (eeprom[40+i] & 0xf000) >> 12

        for i in range(32):
            if accColumn[i]>7 :
                accColumn[i] = accColumn[i] - 16
        
        max = -100000.0
        pxvals = [0.0] * 768        
        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                v = (eeprom[64 + p] & 0x03f0) >> 4 
                if v > 31: v = v - 64
                
                v *= (1<<accRemScale)                
                v = (alphaRef + (accRow[i] << accRowScale) + (accColumn[j] << accColumnScale) + v)                
                v = float(v) / (1<<alphaScale)                
                v = v - self.__cal_tgc * (self.__cal_cpAlpha[0] + self.__cal_cpAlpha[1])/2
                
                v = self.__SCALE_ALPHA / v
                pxvals[p] = v
                if (v> max):
                    max = v                

        alphaScale = 0
        while max < 32768:
            max *= 2
            alphaScale += 1
        
        for i in range(768):
            v = pxvals[i] * (1<<alphaScale)
            self.__cal_alpha[i] = int(v + 0.5)                
        self.__cal_alphaScale = alphaScale
        print (self.__cal_alpha)


    __cal_offset:array = array('h', bytearray(2 * 768)) #int16 x 768

    def __unpack_offset_parameters(self, eeprom:array):
        self.__print_message("Unpacking pixel Offset values")

        occRow = [0] * 24
        occColumn = [0] * 32

        occRemScale = eeprom[16] & 0x000f
        occColumnScale = (eeprom[16] & 0x00f0) >> 4
        occRowScale = (eeprom[16] & 0xf00) >> 8
        offsetRef = eeprom[17]
        if offsetRef > 32767: offsetRef -= 65536

        for i in range(6):
            p = i*4
            occRow[p + 0] = (eeprom[18+i] & 0x000f)
            occRow[p + 1] = (eeprom[18+i] & 0x00f0) >> 4
            occRow[p + 2] = (eeprom[18+i] & 0x0f00) >> 8
            occRow[p + 3] = (eeprom[18+i] & 0xf000) >> 12

        for i in range(24):
            if occRow[i]>7 :
                occRow[i] = occRow[i] - 16

        for i in range(8):
            p = i*4
            occColumn[p + 0] = (eeprom[24+i] & 0x000f)
            occColumn[p + 1] = (eeprom[24+i] & 0x00f0) >> 4
            occColumn[p + 2] = (eeprom[24+i] & 0x0f00) >> 8
            occColumn[p + 3] = (eeprom[24+i] & 0xf000) >> 12

        for i in range(32):
            if occColumn[i]>7 :
                occColumn[i] = occColumn[i] - 16

        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                v:int = (eeprom[64 + p] & 0xfc00)>>10
                if v>31: v = v - 64
                v *= (1<<occRemScale)
                v = (offsetRef + (occRow[i] << occRowScale) + (occColumn[j] << occColumnScale) + v)
                self.__cal_offset[p] = v


    __cal_kta:array = array('b', bytearray(768)) #int8 x 768

    def __unpack_kta_pixel_calibration(self, eeprom:array):
        self.__print_message("Unpacking Kta pixel calibration values")

        ktaRc = [0,0,0,0]
        
        v = (eeprom[54] & 0xff00) >> 8
        if v > 127: v = v - 256
        ktaRc[0] = v

        v = (eeprom[54] & 0xff) 
        if v > 127: v = v - 256
        ktaRc[2] = v

        v = (eeprom[55] & 0xff00) >> 8
        if v > 127: v = v - 256
        ktaRc[1] = v

        v = (eeprom[55] & 0xff)
        if v > 127: v = v - 256
        ktaRc[3] = v

        ktaScale1 = ((eeprom[56] & 0x00f0) >> 4) + 8
        ktaScale2 = (eeprom[56] & 0xf)

        pxVals = [0.0] * 768
        max = -100000            
        for i in range(24):
            for j in range(32):
                p = 32 * i + j
                split = (2*(int(p/32) - int(p/64)*2) + p%2)
                
                v = (eeprom[64 + p] & 0xe) >> 1
                if v>3 : v = v - 8
                v = v * (1<<ktaScale2)
                v = ktaRc[split] + v
                v = v / (1<<ktaScale1)
                if v > max: max = v
                pxVals[p] = v
        
        ktaScale1 = 0
        while(max < 64):
            max *= 2
            ktaScale1 += 1
        
        for i in range(768):
            v = pxVals[i] * (1<<ktaScale1)
            if v<0:
                v = int(v-0.5)
            else:
                v = int(v+0.5)
            self.__cal_kta[i] = v
        
        self.__cal_kta_scale = ktaScale1


    __cal_kv:array = array('b', bytearray(768)) #int8 x 768
                
    def __unpack_kv_pixel_calibration(self, eeprom:array):
        self.__print_message("Unpacking Kv pixel calibration values")

        kvT = [0,0,0,0]
            
        v = (eeprom[52] & 0xf000) >> 12
        if v > 7: v = v - 16
        kvT[0] = v

        v = (eeprom[52] & 0x0f00) >> 8
        if v > 7: v = v - 16
        kvT[2] = v

        v = (eeprom[52] & 0x00f0) >> 4
        if v > 7: v = v - 16
        kvT[1] = v

        v = (eeprom[52] & 0x000f)
        if v > 7: v = v - 16
        kvT[3] = v

        kvScale = (eeprom[56] & 0x0f00) >> 8

        pxVals = [0.0] * 768
        max = -100000 
        for i in range(24):
            for j in range(32):
                p = 32*i + j
                split = 2* (int(p/32) - int(p/64)*2) + p%2
                v = kvT[split]
                v = v / (1<<kvScale)
                av = abs(v)
                if av > max: max = av
                pxVals[p] = v

        kvScale = 0
        while (max < 64):
            max *= 2
            kvScale += 1
        print (kvScale)
        for i in range(768):
            v = pxVals[i] * (1<<kvScale)
            if v<0:
                v = int(v-0.5)
            else:
                v = int(v+0.5)
            self.__cal_kv[i] = v
        self.__cal_kv_scale = kvScale

    
    __cal_ilChessC:array = array('f', bytearray(4 * 3)) #float x 3

    def __unpack_cil_parameters(self, eeprom:array):
        self.__print_message("Unpacking CIL calibration")

        calMode = (eeprom[10]& 0x0800) >> 4
        calMode = calMode ^ 0x80
        self.__cal_cal_mode_ee = calMode
        
        v = (eeprom[53] & 0x003f)
        if v>31: v = v-64
        self.__cal_ilChessC[0] = v/16.0

        v = (eeprom[53] & 0x07c0) >> 6
        if v>15: v = v-32
        self.__cal_ilChessC[1] = v/2.0
        
        v = (eeprom[53] & 0xf800) >> 11
        if v>15: v = v-32
        self.__cal_ilChessC[2] = v/8.0
            







