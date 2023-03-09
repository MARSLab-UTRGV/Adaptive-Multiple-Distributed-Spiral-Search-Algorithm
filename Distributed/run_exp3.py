import os, time
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
import shutil, sys
np.set_printoptions(suppress=True)


def Experiment3(rc):

    
    xml_f100 = './experiments/Experiment3_food100.xml'
    xml_f150 = './experiments/Experiment3_food150.xml'
    xml_f200 = './experiments/Experiment3_food200.xml'
    xml_f250 = './experiments/Experiment3_food250.xml'
    xml_f300 = './experiments/Experiment3_food300.xml'
    
    run_count = rc
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Distributed Experiment 3: 100 food,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_f100}')
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Distributed Experiment 3: 150 food,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_f150}')

    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Distributed Experiment 3: 200 food,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_f200}')
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Distributed Experiment 3: 250 food,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_f250}')

    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Distributed Experiment 3: 300 food,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_f300}')


if __name__ == '__main__':

    if len(sys.argv) != 2:
        print('Usage: python3 run.py <run_count>')
        sys.exit(1)

    rc = int(sys.argv[1])

    Experiment3(rc)
