classdef kalman
    %KARMAN Summary of this class goes here
    %   Detailed explanation goes here

    properties (Access = public)
        P (4,4) double
        X (4, 1) double
        status logical = false;
        std_regs (4,:) double
        idx uint16
    end

    properties (Access = private)
        std_angular_v double
        std_v double
        std_WGN double
        cov_w double
        cov_v double
        cov_l double
        initial_X (4,1) double
        initial_P (4,4) double
    end

    methods (Access = public)
        function self = kalman(X0,std, cov)
            arguments
                % X0 (3,1) double = [0;0;0];
                std (3,1) double = [deg2rad(3);0.05;0.15];
                cov (3,1) double = zeros(1,3);
            end
            
            self.std_angular_v = std(1);
            self.std_v = std(2);
            self.std_WGN = std(3);

            self.cov_w = cov(1);
            self.cov_v = cov(2);
            self.cov_l = cov(3);

            self.initial_P = diag([cov 0]);
            self.P = zeros(4);
            
            self.std_regs = zeros(4, 512);
            self.idx = 0;

        end

        function est = estimate(self, vw, t)
            arguments
                self;
                vw (2,1) double;
                t (2,1) uint32;
            end
            t = double(t);
            dt = (t(2) - t(1)) * 1e-4;
            self.status = true;

            v = vw(1);
            w = vw(2);

            curr = self.X(1:4);
            heading = curr(3);
            prediction = curr + dt * [v*cos(heading); v *sin(heading); w; 0];
            sigma_av = (self.std_angular_v*dt)^2;
            sigma_v = (self.std_v*dt)^2;
            Q = diag([sigma_av sigma_av sigma_v 0]);

            F = [1 0 -v*sin(heading)*dt 0;
                 0 1 v*cos(heading)*dt 0;
                 0 0 1 0;
                 0 0 0 1];

            self.P = F * self.P * F' + Q;
            self.X = prediction;
            est = [obj.X(1:3); t(2)*1e-4];

            % slef.addToStdRegister(obj.P(1:3, 1:3), t(2));
        end
        
        function [X, P] = update(self, OOI_reference, OOI_measured)
            arguments
                self;
                OOI_reference (2, :) double;
                OOI_measured (2,:) double;
            end
            R = self.std_WGN^2;
            measured_z = sqrt( (OOI_measured(1,:) - self.X(1)).^2 + (OOI_measured(2,:) - self.X(2)).^2);

            for i = 1:len(OOI_measured)
                expected_distance = sqrt((OOI_reference(1,i) - self.X(1)).^2 + (OOI_reference(2,i) - self.X(2)).^2);

                H = [(self.X(1) - OOI_reference(1,i))/expected_distance, ... 
                    (obj.X(2) - OOI_reference(2,i))/expected_distance, 0, 0];
                
                K = self.P * H' * inv(H * self.P * H' + R);
                
                self.X = self.X + K * (measured_z(i) - expected_distance); 

                self.P = (eye(4) - K*H) * self.P;
                
                self.X(4) = clip(self.X(4), -1, 1); % bias
            end

            X = self.X; P = self.P;
        end
   
    end
    
    methods (Access = private)
        function addReg(self, P, t)
            arguments
                self;
                P (3,3) double;
                t double;
            end

            stds = [sqrt(P(1,1));sqrt(P(2,2));sqrt(P(3,3))];
            i = self.idx + 1;
            if (i > size(self.std_regs,2))
                cat(2, self.std_regs, zeros(4, 512));
            end
            self.std_regs(:, i) = [t; stds];
            self.idx = i;
        end
    end

end