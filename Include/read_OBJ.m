function [V,S] = read_OBJ(filename)
  % Reads a 2D .obj mesh file and outputs the vertex and segment list
  % Input:
  %  filename  string of obj file's path
  %
  % Output:
  %  V  number of vertices x 3 array of vertex positions
  %  S  is cell array of shape struct containing line segments
  %
  V = zeros(0,3);
  S = {};
  segs = [];
  fid = fopen(filename,'rt');
  line = fgets(fid);
  while ischar(line)
      switch (line(1))
          case 'v'
              vertex = sscanf(line,'v %f %f %f');
              V = [V; vertex'];
          case 'o'
              if(~isempty(segs))
                  shape.segments = segs;
                  S = [S, shape];
              end
              shape.name = sscanf(line(3:end), '%s');
              segs = [];
          case 'l'
              polyline = sscanf(line(3:end), '%d');
              for i=2:numel(polyline)
                  segs = [segs; [polyline(i-1) polyline(i)]];
              end
      end
      line = fgets(fid);
  end
  if(~isempty(segs))
      shape.segments = segs;
      S = [S, shape];
  end
fclose(fid);
end