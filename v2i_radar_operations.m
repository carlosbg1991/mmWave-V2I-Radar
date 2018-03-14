% Input parameters:
% - nSlot is the slot number in the simulation
% - rPATT is the Radar pattern. The system only performs radar
%   operations if trPATT(nSlot) is different than 0. The number 
%   indicates the sector over which radar operations are performed.
% - sInt contains the edges of the spatial sectors
% - loc is the real location of all the cars
% Output parameters:
% - vEst is the vector of estimated velocities.
% - locSys is the vector with the estimated location. Serves to predict
%   car's locations using the vEst at every slot.
% - tEst contains the information as to when the system has
%   detected/updated the car speed and location
function [vEst,tEst,locSys] = v2i_radar_operations(nSlot,tSym,rPATT,loc,sInt,vInt,tInt,e_radar,vEst,tEst,locSys,DEBUG)
    % Determine the sector the radar is covering
    sector = rPATT(nSlot);
    inferiorX = sInt(sector);
    superiorX = sInt(sector + 1);
    % Determine which cars are within that sector
    cIDList = find((loc >= inferiorX) & (loc <= superiorX));
    if ~isempty(cIDList)
        for cID = (cIDList.')
            idxInt = find(tInt{cID} <= tSym, 1, 'last');
            measError = (2*e_radar)*rand(1) - e_radar;  % Radar error in m/s
            vnew = vInt{cID}(idxInt) + measError;
            vEst{cID} = [vEst{cID} vnew];  % Estimation of velocity with error.
            tEst{cID} = [tEst{cID} tSym];  % Instant when detection ocur.
            locSys(cID) = loc(cID);  % Estimation of location has no error.
            if DEBUG
              fprintf('Car %d detected at %d at location %d\n',cID,tSym,loc(cID));
              fprintf('Radar boundaries: < %.3f , %.3f >\n',inferiorX,superiorX);
            end
        end
    end
end