classdef policyEXP3 < Policy
    %POLICYEXP3 This is a concrete class implementing EXP3.
    
    properties
        % Define member variables
        nbActions   % number of bandit actions
        weights     % weights of each action
        lastAction  % last chosen action
        round       % current round number
    end
    
    methods

        function init(self, nbActions)
            % Initialize member variables
            self.nbActions = nbActions;
            self.weights = ones(1, nbActions);
            self.weights = self.weights / nbActions;
            self.round = 1;
        end
        
        function action = decision(self)
            % Choose an action
            
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
            % reward is the reward of the chosen action
            % update internal model
            
            % First we create the loss vector for GWM
            lossScalar = 1 - reward; % This is loss of the chosen action
            lossVector = zeros(1,self.nbActions);          
            lossVector(self.lastAction) = lossScalar / self.weights(self.lastAction);
            
            % Do more stuff here using loss Vector
            eta = sqrt(log(self.nbActions)/ (self.round * self.nbActions));
            loss_factor = exp(-eta .* lossVector);
            self.weights = self.weights .* loss_factor;
        end        
    end
end