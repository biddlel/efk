classdef vehicle < handle
    properties (Access = public)
        api
        helpers struct
        menu app
        file string
        params table
        status string = "Not initalised"
        p parallel.BackgroundPool
    end

    % observers
    properties (Access = private)
        UpdateFileLsn event.listener
        ExitLsn event.listener
        SpeedUp event.listener
        SlowDown event.listener
        ResetPlayback event.listener
        Playback event.listener
       
        triggerFileUpdate logical = false
        exitFlag logical = false
        speed double = 1;
        resetFlag logical = false;
        playbackFlag logical = false; 
     
    end
    
    % 
    properties (Access = private)
        t0 double = 0
        X (3,1) double = zeros(3,1)
        vw (2,1) double= zeros(2,1)
        imu_index double = 1
        lidar_index double = 1
        loopFlag logical = false;
        cfg struct

    end

    properties(SetObservable = true)
        positions (3, :) double
        lidarScans (1, :) double
        positionErr (4, :) double
    end

    methods
        function obj = vehicle(parameters)
            arguments
                parameters.len double = 1;
                parameters.gain double {mustBePositive} = 1;
                parameters.bias double = 0;
                parameters.file string = "";
            end
            
           obj.status = "starting init";
           obj.api = APImtrn4010_v01();
           obj.file = parameters.file;

           p = [parameters.len, parameters.gain, parameters.bias];
           p_names = ["len", "gain", "bias"];
           obj.params = array2table(p, "VariableNames",p_names);
        
           obj.positions = zeros(3, 4096);
           obj.lidarScans = zeros(1,4096);
           obj.positionErr = zeros(4, 4096);
        
           obj.imu_index = 1;
           obj.lidar_index = 1;
           obj.status = "init";

           obj.helpers = struct( ...
               "ReadNextEvent", obj.api.RdE, ...                           % :read next event
                "ConvertRangestoXY", obj.api.b.ConvertRangestoXY, ...       % :convert scan ranges (polar) to carterian points in LiDAR's CF (*)
                "GetCurrentGT", obj.api.gcgt, ...                         % : get current actual pose.
                "AnimateShape", obj.api.AniShape, ...                     % : helper function for basic animation
                "GetRangeAndIntensityFromRawScan", obj.api.b.GetRangeAndIntensityFromRawScan... % parses raw LidAR scans. (*)
           );

           obj.p = backgroundPool;

        end
        
        function loadFile(obj)
            arguments
                obj vehicle
            end

            dataFile = obj.file();
            
            obj.file = "./datasets/" + string(dataFile);
            obj.status = "Loading file: " + obj.file;
            
            r = obj.api.b.LoadDataFile(obj.file);      
            if r.ok < 1, return ; end 
            obj.api.b.Rst();
            obj.status = "Loaded file: " + obj.file;
            
            nL=r.n(2);  % number of LiDAR events
            nI=r.n(3);  % number of normal events
            obj.positions = zeros(3, nI);
            obj.lidarScans = zeros(1, nL);
            obj.positionErr = zeros(4, nL);
            obj.imu_index = 1;
            obj.lidar_index = 1;
            info = obj.api.b.GetInfo();
            obj.X = info.pose0;

            obj.cfg = struct(...
                "LiDAR1", info.LidarsCfg.Lidar1, ...
                "LiDAR2", info.LidarsCfg.Lidar2 ...
            );

            obj.t0 = 0;
        end

        function configParams(obj, params)
            arguments 
                obj vehicle
                params.len double
                params.gain double
                params.bias double
            end 
            obj.params.len = params.len;
            obj.params.gain = params.gain;
            obj.params.bias = params.bias;
        end

        function initMenu(obj)
            obj.menu = app();
            obj.menu.init();

        end

        function help =run(obj)
            obj.initMenu();
            obj.fileReach();
            obj.loadFile();
            
            obj.UpdateFileLsn = addlistener(obj.menu, "FileChange", @(src, evnt)fileUpdate(obj, src, evnt));
            obj.ExitLsn = addlistener(obj.menu, "Exit", @(src, evnt)exitEvent(obj, src, evnt));
            obj.SpeedUp = addlistener(obj.menu, "SpeedUp", @(src, evnt)speedUp(obj, src, evnt));
            obj.SlowDown = addlistener(obj.menu, "SlowDown", @(src, evnt)slowDown(obj, src, evnt));
            obj.ResetPlayback = addlistener(obj.menu, "ResetPlayback", @(src, evnt)resetSwitch(obj, src, evnt));
            obj.Playback = addlistener(obj.menu, "Playback", @(src, evnt)playbackSwitch(obj, src, evnt));
            
            obj.loadMenu();

            help = obj.menu;
        end
        
        function fileReach(obj)
            obj.file = obj.menu.getFileName();
        end

        function fileUpdate(obj, ~, ~)
            obj.fileReach();
            str = "File changed to " + obj.file;
            disp(str);
            obj.triggerFileUpdate = true;
        end

        function exitEvent(obj, ~, ~)
            obj.exitFlag = true;
            disp("Exit button raised");
            obj.delete();
        end

        function speedUp(obj, ~, ~) 
            obj.speed = obj.speed + 0.05;
        end

        function slowDown(obj, ~, ~)
            obj.speed = obj.speed - 0.05;
        end

        function computeEvent(obj)
            idx = obj.imu_index;
            obj.positions(:, idx) = obj.X;

            obj.menu.updateGCFPlot(obj.positions, idx);

            e = obj.helpers.ReadNextEvent();
            t = double(e.t)*0.0001;
            dt = t-obj.t0;
            obj.t0=t;
            obj.computePose(dt);
            
            switch(e.ty)
            
                case 0   % -----------------------------  "end of trip" event -------------------- 
                    % data type = 0 --> END, no more events. end of trip. so END LOOP.
                    disp('Found the END');
                    obj.exitFlag = true;    
                case 1
                    % d = e.d;

                    poseGTNow = obj.helpers.GetCurrentGT();
                    obj.computePoseError(poseGTNow);
                    % obj.computeLiDAR(d);
                    obj.lidar_index = obj.lidar_index + 1;

                    obj.menu.updateErrPlot(obj.positionErr, obj.lidar_index);
                case 2
                    obj.vw = e.d;
                    obj.imu_index = obj.imu_index + 1;

                
                otherwise %-----------------------------
                % ? % we do not consider other sensors in Project 1, so we ignore those. 
            end

        end

        function computePose(obj, dt)
            % curr = [ x; y; heading (rad)] 
            h1 = obj.X(3);
            v = obj.vw(1);
            w = obj.vw(2);
            
            dpose =[v*cos(h1);
                    v*sin(h1);
                    w ];
            obj.X = obj.X + dt.*dpose;
        end

        function computePoseError(obj, gt)
            idx = obj.lidar_index + 1;
            
            x = gt(1) - obj.X(1);
            y = gt(2) - obj.X(2);
            heading = gt(3) - obj.X(3);
            total = sqrt(x^2 + y^2);

            obj.positionErr(:, idx) = [x, y, heading, total];
        end

        function loadMenu(obj)
            UsefulInfo = obj.api.b.GetInfo();
            GT = obj.api.b.GetGroundTruth();
            obj.menu.load(UsefulInfo, GT);
        end

        % function computeLiDAR(obj, data)
        % 
        % end
        
        function loop(obj)
            obj.loopFlag = true;
            while(obj.playbackFlag && ~obj.exitFlag) 
                obj.computeEvent();
                if (obj.triggerFileUpdate)
                    obj.fileChange();
                end
                if (obj.resetFlag)
                    reset();
                end


                pause(0.025*obj.speed);
            end
            obj.loopFlag = false;
        end

        function playbackSwitch(obj, ~, ~)
            disp("playback toggle!!!")
            if (obj.playbackFlag)
                obj.playbackFlag = ~obj.playbackFlag;
            else
                obj.playbackFlag = ~obj.playbackFlag;
                
                if (~obj.loopFlag)
                    f = parfeval(obj.p, @(self) obj.loop, 2, obj); % implementing parfeval
                end
            end
        end

        function resetSwitch(obj, ~, ~)
            obj.resetFlag = ~obj.resetFlag;
            obj.reset();
        end

        function reset(obj) 
            obj.loadFile();
            obj.loadMenu();
            obj.resetFlag = ~obj.resetFlag;
            obj.playbackFlag = false;
        end

        function fileChange(obj)
            obj.triggerFileUpdate = ~obj.triggerFileUpdate;
            obj.reset();
        end  
        
    end

    methods(Access = private)
        function delete(obj)
            delete(obj.p)
            % delete api
            obj.menu.delete();
            delete(obj);
        end
        
    end
end