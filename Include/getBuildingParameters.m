function [droneAltitudes, floorIndices, groundAltitude, buildingHeight, dummyPosition]  = getBuildingParameters(building_name, coverage_type)

switch building_name
    case 'Academic'
        buildingHeight = 32.0;
        groundAltitude = -76.0;
        dummyPosition = [722292.947,3159642.171];
        if strcmp(coverage_type, 'walls')
            droneAltitudes = [14.5, 23.5]; % Ground is zero for the drone
            floorIndices = [3, 5]; % Floor indices corresponding to drone heights
        elseif strcmp(coverage_type, 'roof')
            droneAltitudes = [buildingHeight + 8.0];
            floorIndices = [1]; % For roof, take the floor with largest area
        else
            error('Unknown coverage type.');
        end
    otherwise
        error(['Parameters for building %s are not available!\n', ...
            'Choices are: \n', ...
            'Academic \n', ...
            'Residence \n', ...
            ], building_name);
end
