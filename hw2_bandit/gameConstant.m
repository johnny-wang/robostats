classdef gameConstant < Game
    %GAMECONSTANT This is a concrete class defining a game where rewards a
    %   are fixed.
    % Nothing needs to be filled up here - this is a complete example
    
    % Only the constructor is defined here
    methods
        function self = gameConstant()
            %   The constructor takes no inputs for this class
            
            self.nbActions = 2;         % 2 actions
            self.totalRounds = 1000;    % 1000 time steps
            self.tabR = repmat([0.2; 0.8], 1, self.totalRounds); % table of rewards
            self.N = 0; % the current round counter is initialized to 0    
        end
        
    end    
end

