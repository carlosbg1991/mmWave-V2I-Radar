function [vEst,loc] = v2i_radar_operations(nSlot,rPATT,loc,sInt,vInt,tInt,r_error,vEst)
    % Input parameters:
    % - nSlot is the slot number in the simulation
    % - rPATT is the Radar pattern.
    % - sInt contains the edges of the spatial sectors
    % - loc is the location of all the cars (real)
    % - vEst is the vector of estimated velocities
    
    % Decide wheather we need to perform radar operations
    if rPATT(nSlot) ~=0
        % Determine the sector the radar is covering
        sector = rPATT(nSlot);
        inferiorX = sInt(sector);
        superiorX = sInt(sector + 1);
        % Determine which cars are within that sector
        cIDList = find((loc < inferiorX) & (loc < superiorX));
        for cID = cIDList
            idxInt = find(tInt{cID} <= tSym, 1, 'last');
            vnew = vInt{cID}(idxInt) + r_error*rand(1);
            vEst{cID} = [vEst{cID} vnew];
        end
    end
end