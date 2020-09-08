% 
clear all
close all
loadloggeddata('datalog');

drawlogfromtag('datalog.mat','N1_sim')
drawlogfromtag('datalog.mat','N2_sim')
drawlogfromtag('datalog.mat','EGT_sim')
drawlogfromtag('datalog.mat','FF_sim')
drawlogfromtag('datalog.mat','Thrust')

