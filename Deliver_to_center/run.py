import os, time
import matplotlib.pyplot as plt
from datetime import datetime
import numpy as np
import shutil, sys
np.set_printoptions(suppress=True)

F_MDSA_C = "./MDSA_Centralized"
F_MDSA_D = "./MDSA_Distributed"
F_DDSA = "./DDSA"

COLLISION_LIST = []

# reads the collsion list file and parses it into a list of tuples where each tuple is of the form (x,y) where x is the x coordinate and y is the y coordinate
def readCollisionList(fname):
    global COLLISION_LIST
    COLLISION_LIST.clear()
    with open(fname) as f:
        for line in f.readlines():
            data = line.strip().split(',')
            if data[0] == 'Collected per collision':
                continue
            for d in data:
                if d == '': continue
                coord = d.strip().split(' ')
                COLLISION_LIST.append((float(coord[0]), float(coord[1])))

def createHexHeatMap():

    readCollisionList('./results/Exp1_8x8_r8_rg4_f128_t12_MDSA-C-CollisionLocations.txt')

    coordinates = COLLISION_LIST

    # Extract the x and y values from the coordinates
    x = [c[0] for c in coordinates]
    y = [c[1] for c in coordinates]

    # Create the hexbin plot
    plt.hexbin(x, y, gridsize=20, cmap='Reds')

    # Set the title and labels
    plt.title('Heatmap of Coordinates')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    plt.xticks(range(-4,5))
    plt.yticks(range(-4,5))

    plt.colorbar()

    plt.tight_layout()

    plt.savefig('./heatmap_test.png')

def createScatterHeatMap():

    readCollisionList('./results/Exp1_8x8_r8_rg4_f128_t12_MDSA-C-CollisionLocations.txt')

    collisions = COLLISION_LIST

    # Create a 2D histogram of the collisions
    hist, xedges, yedges = np.histogram2d([c[0] for c in collisions], [c[1] for c in collisions], bins=7, range=[[-5, 5], [-5, 5]])

    # Create the pcolormesh plot
    plt.pcolormesh(xedges, yedges, hist.T)

    # Set the title and labels
    plt.title('Collision Density')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')

    # Set the x and y ticks to be from 0 to 6
    plt.xticks(range(-5, 6))
    plt.yticks(range(-5, 6))

    # Add a colorbar
    plt.colorbar()

    # Display the plot
    plt.savefig('heatmap_scatter.png')

def Experiment1(rc):

    xml_rg4 = './experiments/Experiment1_rg4.xml'
    xml_rg16 = './experiments/Experiment1_rg16.xml'
    xml_rg64 = './experiments/Experiment1_rg64.xml'

    run_count = rc
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Cenralized Experiment 1: 4 regions,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_rg4}')
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Cenralized Experiment 1: 16 regions,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_rg16}')
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Cenralized Experiment 1: 64 regions,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_rg64}')

def Experiment2(rc):

    xml_r4 = './experiments/Experiment2_r4.xml'
    xml_r8 = './experiments/Experiment2_r8.xml'
    xml_r16 = './experiments/Experiment2_r16.xml'
    xml_24 = './experiments/Experiment2_r24.xml'
    xml_r32 = './experiments/Experiment2_r32.xml'

    run_count = rc
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Cenralized Experiment 2: 4 robots,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_r4}')
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Cenralized Experiment 2: 8 robots,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_r8}')
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Cenralized Experiment 2: 16 robots,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_r16}')

    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Cenralized Experiment 2: 24 robots,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_24}')
    
    for j in range(run_count):
        time.sleep(0.05)
        print(f'MDSA-Cenralized Experiment 2: 32 robots,  Iteration: {j+1}/{run_count}\n')
        os.system(f'argos3 -c {xml_r32}') 

if __name__ == '__main__':

    # if len(sys.argv) != 2:
    #     print('Usage: python3 run.py <run_count>')
    #     sys.exit(1)

    # rc = int(sys.argv[1])

    # Experiment1(rc)
    # Experiment2(rc)

    createHexHeatMap()

