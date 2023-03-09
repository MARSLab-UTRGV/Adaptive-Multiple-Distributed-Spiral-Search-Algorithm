import os, time
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
import shutil, sys
np.set_printoptions(suppress=True)

def Experiment1(rc):

    xml_rg4 = './experiments/Experiment1_rg4.xml'
    xml_rg16 = './experiments/Experiment1_rg16.xml'
    xml_rg64 = './experiments/Experiment1_rg64.xml'

    run_count = rc
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDBSA-Cenralized Experiment 1: 4 regions,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_rg4}')
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDBSA-Cenralized Experiment 1: 16 regions,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_rg16}')
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDBSA-Cenralized Experiment 1: 64 regions,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_rg64}')

def Experiment2(rc):

    
    xml_r8 = './experiments/Experiment2_r8.xml'
    #xml_r16 = './experiments/Experiment2_r16.xml'
    xml_r14 = './experiments/Experiment2_r14.xml'
    xml_20 = './experiments/Experiment2_r20.xml'
    xml_r26 = './experiments/Experiment2_r26.xml'
	xml_r32 = './experiments/Experiment2_r32.xml'
    
    run_count = rc
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Distributed Experiment 2: 8 robots,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_r8}')
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Distributed Experiment 2: 14 robots,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_r14}')

    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Distributed Experiment 2: 20 robots,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_20}')
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Distributed Experiment 2: 26 robots,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_r26}')

    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Distributed Experiment 2: 32 robots,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_r32}')


if __name__ == '__main__':

    if len(sys.argv) != 2:
        print('Usage: python3 run.py <run_count>')
        sys.exit(1)

    rc = int(sys.argv[1])

    Experiment1(rc)
    Experiment2(rc)
