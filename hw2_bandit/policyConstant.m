classdef policyConstant < Policy
    %POLICYCONSTANT Chooses a random action in the beginning and holds on
    %to it

    properties
        chosenAction
    end
    
    methods

        function init(self, nbActions)
            % randomly choose an action in the begin
            self.chosenAction = randi(nbActions);
        end
        
        function action = decision(self)
            % keep playing the same action 
            action = self.chosenAction;
        end
        
        function getReward(self, reward)
            % do nothing
        end        
    end
end

