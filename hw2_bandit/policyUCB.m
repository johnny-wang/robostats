classdef policyUCB < Policy
    %POLICYUCB This is a concrete class implementing UCB.

        
    properties
        % Member variables        
        nbActions   % number of bandit actions
        conf        % vector of confidence (nbActions x 1)
        selection   % vector of selection counter (nbActions x 1)
        lastAction  % last chosen action
        round       % current round number
        conf_print  % vector to hold confidences'
    end
    
    methods
        function init(self, nbActions)            
            % Initialize member variables
            self.nbActions = nbActions;
            self.conf = ones(nbActions,1) * eps;        % smallest value possible in Matlab
            self.selection = zeros(nbActions,1);
            self.round = 1;
            self.conf_print = ones(nbActions,1);
        end
        
        function action = decision(self)
            % Choose action            
            tmp_action = zeros(self.nbActions,1); % temp vector to store all actions of the round
            % calculate all actions of the round
            for i=1:self.nbActions
                root_term = sqrt(log(self.round)/(2*self.conf(i)));
                tmp_action(i) = self.selection(i)/self.conf(i) + root_term;
            end
            % Get the max value of the actions
            [val, idx] = max(tmp_action);
            action = idx;
            self.lastAction = idx;
            
            % Save confidence to print
            self.conf_print = [self.conf_print tmp_action];
            
            % Update round
            self.round = self.round + 1;            
        end
        
        function getReward(self, reward)
            % reward is scalar
            % Update ucb
            self.selection(self.lastAction) = self.selection(self.lastAction) + reward;
            self.conf(self.lastAction) = self.conf(self.lastAction) + 1;
        end        
    end

end
