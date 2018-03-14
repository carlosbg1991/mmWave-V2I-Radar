% This function computes the location of cars based on the real and
% estimated velocities. The real location is updated for every car. The
% estimated location is only updated for those cars that were detected upon
% the function call.
% Input parameters:
% - loc:     (Vector-double) The real location of the cars.
% - locEst:  (Vector-double) The estimated location of the cars.
% - vEst:    (Vector-cell) The estimated velocities.
% - tEst:    (Vecto-cell) The time at which the detection happened.
% - carsID:  (Vector-double) The ID of every car in the network
% - tInt:    (Vector-cell) The intervals at which cars experience a 
%            reduction in their velocity due to collision.
% - vInt:    (Vector-cell) The velocities for every car.
% - s_radar: (Integer) Number of slots the radar operation takes
% - tSym:    (Double) Elapsed time in the simulation
% - TSLOT:   (Double) Window of time in the simulation. Fixed.
% Output parameters:
% - loc:    (Vector-double) The real location of the cars (updated).
% - locSys: (Vector-double) The estimated location of the cars (updated).
function [loc,locSys] = v2i_update_location(loc,locSys,vEst,tEst,carsID,tInt,vInt,s_radar,tSym,TSLOT,kmul)
    % Update actual car location (known velocities)
    for cID = carsID
        idxInt = find(tInt{cID} <= (kmul*tSym), 1, 'last');
        loc(cID) = loc(cID) + vInt{cID}(idxInt)*(kmul*TSLOT);
    end
    % Update car location (estimation using vEst and tEst) tEst represents
    % the moment at which the car was detected vEst represents the vector
    % of velocities
    carIDToUpdt = find(~cellfun(@isempty,tEst));  % cars to be updated
    for cID = (carIDToUpdt).'
        % We take into account the group of measures that happened
        % consecutivelly to improve the accuracy. That is, the measures
        % that happened within (s_radar*TSLOT) intervals. Otherwise, the
        % system interprets the measure correspond to a period before
        % experiencing a colision. Thus, it shall not be taken into
        % account.
        idxSet = detectMySet(tEst{cID},s_radar*TSLOT);
        % Improved measure by taking the mean of the last measurement block
        vEst_improved = mean(vEst{cID}(idxSet));
        % Elapsed time from last measure
        elapsed = tSym - tEst{cID}(end);
        % Estimated new location
        locSys(cID) = locSys(cID) + (vEst_improved*elapsed);
    end
end

function [idxSet,mySet] = detectMySet(inVect,tWindow)
    % Required number of consecutive numbers following a first one. In
    % other words, blocks of size (N+1).
    % We need to round it up to the 5th decimal to account for unaccuracies
    % in the resolutions. A bug in Matlab, can't deal with it. Keep out.
    x = (round(diff(inVect),5)==tWindow);
    if length(inVect) > 1 && ~isempty(find(x==1, 1))
        % Vector has more than one measure
        f = find([false,x]~=[x,false]);
        if f(end)==length(inVect)
            % There is a block with the last measure
            idxSet = [f(end-1) f(end)];
        else
            % Vector has no block containing the last measure
            idxSet = length(inVect);
        end
    else
        % Vector has only one measure
        idxSet = 1;
    end
    mySet = inVect(idxSet);
end