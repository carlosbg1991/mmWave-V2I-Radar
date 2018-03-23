## Introduction

This project presents a packet scheduler in an Infraestructure-to-Vehicular (I2V) scenario operating in the millimeter wave (mmWave) band using the 802.11ad frames. We build upon the research conducted by prof. Eldar (University of Technion, Israel) where the 802.11ad were proven to be used for radar operations, providing accurate information about the location and velocity of moving objects (i.e. vehicles in a highway).

The code is build as a Discrete-Event-Simulator (DES), where 802.11ad Time Slot (TS) is conceived as an event in the system. Vehicles move along the x-axis at a speed range typical from a highway scenario. Each vehicle is assumed to be synchronized with the network and awaiting to receive data form the Base Station (BS). 

The BS contains the logic where, at each TS, it decides which car to serve based on their application requirements. The beamwidth and the sector over which to transmit is controlled using the estimated location and speed of the vehicle. The BS also controls the operation mode per slot: radar or communication. For radar, it sends dummy 802.11ad frames to estimate location and speed in a given sector. For communicaitons, it sends relevant information to the vehicles.

At the end of the execution, the script plots measurements about throughput, distance-to-BS and measured SNR in a TS-basis.

## Project hierarchy

The project contains several functions, yet few of them are of interest and could be modified by the user:
1. **v2i_main_runnable.m**: The main script in the project that executes the simulator with the configuration parameters in the *v2i_config.m*. Once done, it plots the results.
2. **v2i_config.m**: Contains the main parameter configuration encompassing communications (Modultaion, Beamwidth, etc), radar/comms ratio (TS performing radar operataions vs communications), scheduling policy (Round Robin, Proportional Fairness, Greedy, etc), etcetera.

## Contact

Please, feel free to contact us for any questions or concerns you may have at:

Carlos Bocanegra Guerra
PhD Candidate  
Electrical and Computer Engineering (EECE)  
Northeastern University  
bocanegrac@coe.neu.edu

Guillem Reus Muns  
PhD Student  
Electrical and Computer Engineering (EECE)  
Northeastern University  
guillem.reusmuns@gmail.com  