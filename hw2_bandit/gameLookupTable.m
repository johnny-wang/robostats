classdef gameLookupTable < Game
    %GAMELOOKUPTABLE This is a concrete class defining a game defined by an
    %external input
    
    methods
        function self = gameLookupTable(tabInput, isLoss)
            % Input
            %   tabInput - input table (actions x rewards or losses)
            %   isLoss - 1 if input table represent loss, 0 otherwise            
            
            self.nbActions = size(tabInput,1);   % rows are the actions
            self.totalRounds = size(tabInput,2); % cols are the rounds
            self.N = 0; % the current round counter is initialized to 0
            
            % change table to table of rewards
            if (isLoss == true)
                self.tabR = 1-tabInput;
            else
                self.tabR = tabInput;
            end
            
        end
        
    end
    
end
