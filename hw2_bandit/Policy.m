classdef (Abstract) Policy < handle
    %POLICY This is an abstract class defining a policy.
    %   This class defines common functionality for a policy. The policy is
    %   told the number of actions it can choose from. It has to define
    %   functionality for choosing an action and for updating its belief 
    %   when it gets reward for a chosen action.
    
    %   Do not modify this file. The functions will be implemented in the
    %   concrete class that inherit from this
 
    properties
    end
    
    methods (Abstract)
        init(self, nbActions); 
        % Initialize the policy with number of actions. This function is
        % called before a game is played
        
        a = decision(self); 
        % Choose an action at the current round
        
        getReward(self, reward); 
        % Receive a reward for the chosen action and update internal model.
    end
    
end
