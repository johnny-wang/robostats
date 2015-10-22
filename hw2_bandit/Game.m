classdef (Abstract) Game < handle
    %GAME This is an abstract class defining a game.
    %   This class defines common functionality for a game. It contains 
    %   member variables that define the number of actions, rounds of the
    %   game and the reward for each action at each round. 
    %
    %   It contains common functions for a game such as retrieving the
    %   reward for an action at a time, cumulative reward of best action in
    %   hindsight, etc. It also simulates playing the game given a policy
    %   object. 
    %
    %   Since this Game class is abstract, it is cannot directly be used. A
    %   concrete class that inherits from this Game class and defines
    %   values for the member variables can be used.

    
    % Member variables of the class
    properties
        nbActions   % number of actions
        totalRounds % number of rounds of the game
        tabR        % table of rewards (nbActions x totalRounds)
        N           % counter for the current round of the game 
    end
    
    % Member functions of this class
    methods      
 
        function [reward, action, regret] = play(self, policy)
            %   The function simulates the entire game where at each time
            %   step it class the policy to get an action, evaluates reward
            %   for that action and also evaluates regret for the policy
            %   till the timestep.
            %   Input:
            %       policy: An object of the policy class
            %   Output:
            %       reward: Rewards obtained by the policy (1xtotalRounds)
            %       action: Actions chosen by the policy (1xtotalRounds)
            %       regret: Regret of the policy (1xtotalRounds)
            
            policy.init(self.nbActions);            % Initialize the policy
            
            % Initialize rewards, actions, regret to 0
            reward = zeros(1, self.totalRounds);    
            action = zeros(1, self.totalRounds);
            regret = zeros(1, self.totalRounds);
            
            % Play the game
            for t = 1:self.totalRounds            
                self.N = self.N + 1; % Update the current round counter
                action(t) = policy.decision(); % Get action from policy
                reward(t) = self.reward(action(t)); % Get reward for policy
                regret(t) = self.cumulativeRewardBestActionHindsight() - sum(reward); % Get regret
                policy.getReward(reward(t)); % Update policy
            end
        end
        
        function resetGame(self)
            %   Resets the current counter to the beginning
            self.N = 0;
        end
        
        function r = reward(self, a)
            %   Returns the reward for an action a 
            r = self.tabR(a, self.N);              
        end
        
        %% FILL IN THE FUNCTION BELOW
        function r = cumulativeRewardBestActionHindsight(self)
            %   Returns the cumulative reward of the best fixed action in 
            %   hindsight
            cum_reward = sum(self.tabR(:,1:self.N), 2);            
            r = max(cum_reward);               
        end
        
    end    
end
