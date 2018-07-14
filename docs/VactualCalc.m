clc
clear all
close all

RPM = 60;

v = (RPM/60)*16*200;
Fclk = 16e6;

VACTUAL = v/((Fclk/2)/(2^23))
