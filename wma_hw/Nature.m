classdef Nature
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        observation = []
        label = 0
        e_prediction = []
        final_prediction = 0
        weights = []
    end
    
    methods
        %% Get functions
        % get the observation
        function obs = get_observation(obj)
            obs = obj.observation;
        end
        % Get the label
        function lab = get_label(obj)
            lab = obj.label;
        end
        function [weights] = get_weights(obj)
            weights = obj.weights;
        end
        %% Set functions
        % set the expert predictions we get from learner
        function set_expert_prediction(obj, e_pred)
            obj.e_prediction = e_pred;
        end
        % set the final prediction
        function set_final_prediction(obj, pred)
            obj.final_prediction = pred;
        end
        % set the weights of the learner
        function set_weights(obj, wts)
            obj.weights = wts;
        end
    end
    
end

