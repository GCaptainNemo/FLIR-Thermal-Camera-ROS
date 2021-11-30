import json
import subprocess
from PIL import Image
import re
import matplotlib.pyplot as plt
import numpy as np
import io
from math import sqrt, exp, log


class Converter:
    def __init__(self, img_file_dir, exiftool_path="exiftool"):
        self.img_file_dir = img_file_dir
        self.exiftool_path = exiftool_path

        self.thermal_np = self.get_raw_img()
        self.meta = self.get_meta_data()



    def get_raw_img(self):
        thermal_img_bytes = subprocess.check_output([self.exiftool_path, "-RawThermalImage", "-b", self.img_file_dir])
        thermal_img_stream = io.BytesIO(thermal_img_bytes)

        thermal_img = Image.open(thermal_img_stream)
        thermal_np = np.array(thermal_img)
        # print("thermal_np.shape = ", thermal_np.shape)
        # plt.imshow(thermal_np)
        # plt.show()
        return thermal_np

    def get_meta_data(self):
        meta_json = subprocess.check_output(
            [self.exiftool_path, self.img_file_dir, '-Emissivity', '-SubjectDistance', '-AtmosphericTemperature',
             '-ReflectedApparentTemperature', '-IRWindowTemperature', '-IRWindowTransmission', '-RelativeHumidity',
             '-PlanckR1', '-PlanckB', '-PlanckF', '-PlanckO', '-PlanckR2', '-j'])
        meta = json.loads(meta_json.decode())[0]
        print("meta thermal = ", meta)
        return meta

    def convert(self):
        if 'SubjectDistance' in self.meta:
            subject_distance = Converter.extract_float(self.meta['SubjectDistance'])
        else:
            subject_distance = 1
        raw2tempfunc = np.vectorize(lambda x: Converter.raw2temp(x, E=self.meta['Emissivity'], OD=subject_distance,
                                                                    RTemp=Converter.extract_float(
                                                                        self.meta['ReflectedApparentTemperature']),
                                                                    ATemp=Converter.extract_float(
                                                                        self.meta['AtmosphericTemperature']),
                                                                    IRWTemp=Converter.extract_float(
                                                                        self.meta['IRWindowTemperature']),
                                                                    IRT=self.meta['IRWindowTransmission'],
                                                                    RH=Converter.extract_float(
                                                                        self.meta['RelativeHumidity']),
                                                                    PR1=self.meta['PlanckR1'], PB=self.meta['PlanckB'],
                                                                    PF=self.meta['PlanckF'],
                                                                    PO=self.meta['PlanckO'], PR2=self.meta['PlanckR2']))
        thermal_np = raw2tempfunc(self.thermal_np)
        return thermal_np
    

    @staticmethod
    def raw2temp(raw, E=1, OD=1, RTemp=20, ATemp=20, IRWTemp=20, IRT=1, RH=50, PR1=21106.77, PB=1501, PF=1, PO=-7340,
                 PR2=0.012545258):
        """
        convert raw values from the flir sensor to temperatures in C
        # this calculation has been ported to python from
        # https://github.com/gtatters/Thermimage/blob/master/R/raw2temp.R
        # a detailed explanation of what is going on here can be found there
        """

        # constants
        ATA1 = 0.006569
        ATA2 = 0.01262
        ATB1 = -0.002276
        ATB2 = -0.00667
        ATX = 1.9

        # transmission through window (calibrated)
        emiss_wind = 1 - IRT
        refl_wind = 0

        # transmission through the air
        h2o = (RH / 100) * exp(1.5587 + 0.06939 * (ATemp) - 0.00027816 * (ATemp) ** 2 + 0.00000068455 * (ATemp) ** 3)
        tau1 = ATX * exp(-sqrt(OD / 2) * (ATA1 + ATB1 * sqrt(h2o))) + (1 - ATX) * exp(
            -sqrt(OD / 2) * (ATA2 + ATB2 * sqrt(h2o)))
        tau2 = ATX * exp(-sqrt(OD / 2) * (ATA1 + ATB1 * sqrt(h2o))) + (1 - ATX) * exp(
            -sqrt(OD / 2) * (ATA2 + ATB2 * sqrt(h2o)))

        # radiance from the environment
        raw_refl1 = PR1 / (PR2 * (exp(PB / (RTemp + 273.15)) - PF)) - PO
        raw_refl1_attn = (1 - E) / E * raw_refl1
        raw_atm1 = PR1 / (PR2 * (exp(PB / (ATemp + 273.15)) - PF)) - PO
        raw_atm1_attn = (1 - tau1) / E / tau1 * raw_atm1
        raw_wind = PR1 / (PR2 * (exp(PB / (IRWTemp + 273.15)) - PF)) - PO
        raw_wind_attn = emiss_wind / E / tau1 / IRT * raw_wind
        raw_refl2 = PR1 / (PR2 * (exp(PB / (RTemp + 273.15)) - PF)) - PO
        raw_refl2_attn = refl_wind / E / tau1 / IRT * raw_refl2
        raw_atm2 = PR1 / (PR2 * (exp(PB / (ATemp + 273.15)) - PF)) - PO
        raw_atm2_attn = (1 - tau2) / E / tau1 / IRT / tau2 * raw_atm2
        # print("raw = {}".format(raw), E, tau1, IRT, tau2, raw_atm1_attn)
        # print(raw_atm2_attn , raw_wind_attn,  raw_refl1_attn , raw_refl2_attn)
        raw_obj = (raw / E / tau1 / IRT / tau2 - raw_atm1_attn -
                   raw_atm2_attn - raw_wind_attn - raw_refl1_attn - raw_refl2_attn)

        # temperature from radiance
        temp_celcius = PB / log(PR1 / (PR2 * (raw_obj + PO)) + PF) - 273.15
        return temp_celcius
    
    @staticmethod
    def extract_float(dirtystr):
        """
        Extract the float value of a string, helpful for parsing the exiftool data
        :return:
        """
        digits = re.findall(r"[-+]?\d*\.\d+|\d+", dirtystr)
        return float(digits[0])

if __name__ == "__main__":
    cvt = Converter("../../../resources/0004.jpg")
    thermal_img = cvt.convert()
    plt.subplot(121)
    plt.imshow(cvt.thermal_np)

    plt.subplot(122)
    plt.imshow(thermal_img, 'hot')    
    plt.show()


