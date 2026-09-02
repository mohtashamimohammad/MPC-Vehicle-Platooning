%% MPC Vehicle Platooning Reproduction
% Main script for reproducing simulation results

clear;
clc;
close all;

%% Add project paths

projectRoot = fileparts(mfilename('fullpath'));

addpath(genpath(fullfile(projectRoot,'src')));
addpath(genpath(fullfile(projectRoot,'results')));

disp('MPC Platooning Simulation Started')

%% Run simulations

run_project

disp('Simulation Completed Successfully')