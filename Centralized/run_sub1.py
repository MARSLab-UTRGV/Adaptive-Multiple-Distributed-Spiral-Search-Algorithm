import os, time
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
import shutil, sys
np.set_printoptions(suppress=True)

def Experiment2(rc):

    
    xml_r14 = './experiments/Experiment2_r14.xml'
    xml_20 = './experiments/Experiment2_r20.xml'
    xml_r26 = './experiments/Experiment2_r26.xml'
	
    run_count = rc
    
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Cenralized Experiment 2: 14 robots,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_r14}')

    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Cenralized Experiment 2: 20 robots,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_20}')
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Cenralized Experiment 2: 26 robots,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_r26}')


if __name__ == '__main__':

    if len(sys.argv) != 2:
        print('Usage: python3 run.py <run_count>')
        sys.exit(1)

    rc = int(sys.argv[1])

    #Experiment1(rc)
    Experiment2(rc)
