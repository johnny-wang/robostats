classdef policyGWM < Policy
    %POLICYGWM This policy implementes GWM for the bandit setting.
    
    properties
        nbActions % number of bandit actions
        % Add more member variables as needed
        weights     % weights of each action
        lastAction  % last action chosen
        round       % current round number
    end
    
    methods
        
        function init(self, nbActions)
            % Initialize any member variables
            self.nbActions = nbActions;
            
            % Initialize other variables as needed
            % Weights of each action is initialized to 1/number of actions
            self.weights = ones(1, nbActions);
            self.weights = self.weights / nbActions;
            self.round = 1;
        end
        
        function action = decision(self)
            % Choose an action according to multinomial distribution            
            
            % normalize weights
            self.weights = self.weights / sum(self.weights);
            % run random multinomial (returns vector of 0s with one 1)
            idx = find(mnrnd(1,self.weights)==1);
            
            % update member variables
            self.lastAction = idx;
            action = idx;
            self.round = self.round + 1;
        end
        
        function getReward(self, reward)
            % Update the weights
            
            % First we create the loss vector for GWM
            lossScalar = 1 - reward; % This is loss of the chosen action
            lossVector = zeros(1,self.nbActions);
            lossVector(self.lastAction) = lossScalar;
            
            % Do more stuff here using loss Vector
            eta = sqrt(log(self.nbActions)/self.round);  % scalar
            loss_factor = exp(-eta * lossScalar);
            self.weights(self.lastAction) = self.weights(self.lastAction) * loss_factor;
        end        
    end
end

