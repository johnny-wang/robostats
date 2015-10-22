classdef gameGaussian < Game
    %GAMEGAUSSIAN This is a concrete class defining a game where rewards a
    %   are drawn from a gaussian distribution.
    
    methods
        
        function self = gameGaussian(nbActions, totalRounds) 
            % Input
            %   nbActions - number of actions
            %   totalRounds - number of rounds of the game
            
            self.nbActions = nbActions;        % 10 actions
            self.totalRounds = totalRounds;    % 10000 time steps
            self.N = 0; % the current round counter is initialized to 0

            % Construct the table of rewards
            for i=1:nbActions
                mu = rand;       % uniform random [0,1]
                sigma = rand;    % uniform random [0,1]
                x_per_rnd = rand(1,totalRounds);    % [0,1] 'x' sample of each round
                self.tabR(i,:) = normpdf(x_per_rnd, mu, sigma);  % create gaussian distribution of rewards                                
            end                        
        end        
    end    
end

