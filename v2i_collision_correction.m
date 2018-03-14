function [xInt,vInt,tInt] = v2i_collision_correction(cInitTime,cVel,cLoc,NCARS,DEBUG)
    % Contains the intervals at which each car needs to adjust its speed
    % due to collision
    xInt  = num2cell(cLoc);            % Spatial velocity updates (collisions)
	vInt  = num2cell(cVel);            % Vector with velocities (collisions)
    tInt  = num2cell(zeros(NCARS,1));  % Time intervals at which speed is updated
    cPass = cell(NCARS,1);             % Stores the cars each cID has reached
    tCol = 0;  % Initial value forcing the system to enter in the loop
    while ~prod(isinf(tCol))  % Iterate until all collision times are Inf
        [~ , idx] = sort(cInitTime);
        tCol = Inf(NCARS,1);  % Contains the earliest collision time for all cID
        cIDCol = cell(NCARS); % Contains the ID of the car cID collides first
        xIntUpdate = num2cell(Inf(1)); % Contains the space intervals (meters)
        vIntUpdate = num2cell(Inf(1)); % Contains the speeds belonging to 
                                       % the space intervals
        % Check collisions for each car on the road
        for id = 1:NCARS
            cID = idx(id);  % Real Car ID
            temp1 = repmat(cLoc(cID),NCARS,1);  % Location of the current cID
            temp2 = cLoc(idx);                  % Location of the rest
            idxTempCol = find(temp1 <= temp2);  % In the sorted list, cars that
                                                % car with real ID "index" has
                                                % passed
            idxTempCol(idxTempCol<=id) = [];
            idxCol{id} = [idxTempCol];                                 %#ok
            % Contains the ID list that the cID is passing during the
            % simulations
            cIDColList = idx(idxCol{id});
            % For each car that cID has collided, calculate the location
            % and time. Return the earliest collision, the only feasible one.
            if ~isempty(cIDColList)
                [tCol(cID) , xIntUpdate{cID} , vIntUpdate{cID}, cIDCol{cID}] ...
                        = detect_collision(cID,cIDColList,xInt,vInt,cPass{cID});
            end
        end
        % Stop execution if no more collisions are detected
        if prod(isinf(tCol))
            if DEBUG; fprintf('END OF EXECUTION - No more collisions were detected\n'); end
            break;
        end
        % We detect the first collision and we correct the speed
        [~,cID] = min(tCol);  % Earliest collision between any pair of cars
        vInt{cID}  = [vInt{cID} vIntUpdate{cID}];  % Update speed vector
        xInt{cID}  = [xInt{cID} xIntUpdate{cID}];  % Update locations
        tInt{cID}  = [tInt{cID} tCol(cID)];        % Update times
        cPass{cID} = [cPass{cID} cIDCol{cID}];     % Update the car cID has passed
        if DEBUG
            fprintf('car %d reaches car %d at time %.2f at location %.2f\n', ...
                    cID,cIDCol{cID},tCol(cID),xIntUpdate{cID}(1));
        end
        % Update total time to simTarget. The total time is the product
        % within the intervals xInt and the velocity at each interval (like
        % an Integral). we add 0 as to simulate the end of the simulation.
        cInitTime(cID) = sum(diff([xInt{cID} 0])./vInt{cID});
        % Update the status of the cars behind the car that just collided
        [xInt,vInt,cPass,cInitTime] = update_tail_cars_velocities ...
                                (cID,xInt,vInt,tInt,cPass,cInitTime,NCARS,DEBUG);
    end
end

function [tCol , xIntUpdate, vIntUpdate, cIDCol] = detect_collision(cID,cIDColList,xInt,vInt,cPass)
    tColList = Inf(length(cIDColList),1);  % Final Collision time for each car in cIDColList
    xColList = Inf(length(cIDColList),1);  % Final Collision location for each car in cIDColList
    % Iterate over each cIDCol that collided with cID
    for idxIDCol = 1:length(cIDColList)
        cIDCol1 = cIDColList(idxIDCol);
        % Calculate intervals in terms of time (xInt -> tInt)
        vIDList = vInt{cID};
        xIDList = xInt{cID};
        vIntColList = vInt{cIDCol1};
        xIntColList = xInt{cIDCol1};
        tIntID  = cumsum(diff([xIDList 0]) ./ vIDList);
        tIntCol = cumsum(diff([xIntColList 0]) ./ vIntColList);
        % Append time intervals and use it as timestamps (ts)
        tsList = [tIntID tIntCol];
        tsList = sort(tsList);
        % Check at each timestamp if cID has surpassed cIDCol
        mID  = xIDList(1);   % Initial Location of cID
        mCol = xIntColList(1);  % Initial Location of cIDCol
        tsPast = 0;          % Previous timeStamp
        for t = 1:length(tsList)
            % Select speeds for each time stamp
            index = find(tIntID >= tsList(t), 1, 'first');
            vID = vIDList(index);
            index = find(tIntCol >= tsList(t), 1, 'first');
            vCol = vIntColList(index);
            % Calculate where they are in space
            mID1 = mID; mCol1 = mCol;  % Backup in case they collide
            mID = mID + vID * (tsList(t) - tsPast);
            mCol = mCol + vCol * (tsList(t) - tsPast);
            % Calculate if they colided in the time stamp interval
            if mID > mCol  % They have collided in the ts interval
                tColInterval = (mID1 - mCol1)./(vCol - vID);  % Time within time interval
                tColList(idxIDCol) = tsPast + tColInterval;
                xColList(idxIDCol) = mID1 + vID*tColInterval;
%                 fprintf('\t\t%d collides with %d at time %.2f at location %.2f\n', ...
%                     cID, cIDCol1, tColList(idxIDCol), xColList(idxIDCol));
                break;
            end
            tsPast = tsList(t);
        end
    end
    notFound = true;
    while(notFound)
        % We take the minimum since the first collision is the one
        % that causes the speed correction. We return the instant, the location
        % and the car ID
        [tCol , indxCol] = min(tColList);
        if ~ismember(cIDColList(indxCol),cPass) || isinf(tCol)
            notFound = false;
        else
            tColList(indxCol) = Inf(1);  % cID has already passed cIDPass
        end
    end
    % The collision happens out of range, we do not care
    if xColList(indxCol) > 0 ; tCol = Inf(1); end
    % The new space intervals need to contain the updated location vector
    % of cIDCol.
    delta = 0.5;  % Car correction in meters. It represents the car length. 
                  % The car needs to break before reaching the next car
                  % (Physics 101)
    xIntUpdate = xColList(indxCol) - delta;
    % Update the latest velocity of the car that we just crashed. If cIDCol
    % has crashed with someone earlier, it is not reflected in the new
    % speed.
    vIntUpdate = vInt{cIDColList(indxCol)}(end);
    % ID of the car first car that collides with cID
    cIDCol = cIDColList(indxCol);
end

function [xInt,vInt,cPass,cInitTime] = update_tail_cars_velocities(cID,xInt,vInt,tInt,cPass,cInitTime,NCARS,DEBUG)
    someoneToUpdate = true;
    cIDUpdate = [];
    delta = 0.5;  % Car length to be corrected in the collision location
    while someoneToUpdate
        for id = 1:NCARS
            if ismember(cID,cPass{id})
                cIDUpdate = id;  % cIDUpdate is the car that is driving 
                                 % right behind the car updated in the 
                                 % previous iteration
                break;
            else
                cIDUpdate = [];
            end
        end
        if ~isempty(cIDUpdate)  % Check if there is any car to be updated
            % Update Basic parameters for cIDUpdate
            xCol = xInt{cID}(end) - delta;
            xInt{cIDUpdate}  = [xInt{cIDUpdate} xCol];
            vInt{cIDUpdate}  = [vInt{cIDUpdate} vInt{cID}(end)];
            tInt{cIDUpdate}  = [tInt{cIDUpdate} tInt{cID}(end)];
            cPass{cIDUpdate} = [cPass{cIDUpdate} cPass{cID}(end)];
            % Update times
            cInitTime(cIDUpdate) = sum(diff([xInt{cIDUpdate} 0])./vInt{cIDUpdate});
            if DEBUG
                fprintf('\tcar %d updated by car %d at time %.2f at location %.2f\n', ...
                        cIDUpdate,cID,tInt{cIDUpdate}(end),xInt{cIDUpdate}(end));
            end
            % The updated car needs to update its followers
            cID = cIDUpdate;
        else
            someoneToUpdate = false;
        end
    end
end

