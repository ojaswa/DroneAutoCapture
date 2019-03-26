% function [V, TC, N, F, FTC, FN] = readOBJFast( objfile )
% Inputs:
% objfile  path to .obj file
% Outputs:
% V  double matrix of vertex positions  #V by 3
% TC  double matrix of texture coordinats #TC by 2
% N  double matrix of corner normals #N by 3
% F  #F list of face indices into vertex positions
% FTC  #F list of face indices into vertex texture coordinates
% FN  #F list of face indices into vertex normals