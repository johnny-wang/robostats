classdef policyRandom < Policy
    %POLICYRANDOM Chooses a random action at every time step
    
    properties
        nbActions
    end
    
    methods

        function init(self, nbActions)
            % store the number of actions
            self.nbActions = nbActions;
        end
        
        function action = decision(self)
            % randomly choose an action from the set of actions
            action = randi(self.nbActions);
        end
        
        function getReward(self, reward)
            % do nothing
        end        
    end
end

