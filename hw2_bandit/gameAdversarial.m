classdef gameAdversarial<Game
    %GAMEADVERSARIAL This is a concrete class defining a game where rewards
    %   are adversarially chosen.

    methods
        
        function self = gameAdversarial()
            self.nbActions = 2;        % 2 actions
            self.totalRounds = 1000;    % at least 1000 time steps
            self.N = 0; % the current round counter is initialized to 0

            self.tabR = repmat([0 1; 1 0], 1, self.totalRounds/2) % table of rewards
%             % Create table of rewards that is adversarial to UCB's
%             % algorithm
%             conf = ones(self.nbActions,1) * eps;
%             selection = zeros(self.nbActions,1);
%             tmp_action = zeros(self.nbActions,1); % temp vector to store all actions of the round
%             
%             for r=1:self.totalRounds
%                 for i=1:self.nbActions
%                     root_term = sqrt(log(r)/(2*conf(i)));
%                     tmp_action(i) = selection(i)/conf(i) + root_term;
%                 end           
%                 % Get the max value of the actions
%                 [val, idx] = min(tmp_action);
%                 self.tabR(r,i) = idx;
%             end                                    
        end        
    end    
end

