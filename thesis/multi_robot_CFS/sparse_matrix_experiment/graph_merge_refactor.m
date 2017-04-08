function [] = graph_merge_refactor(episodes_per_problem, matrix_id, debug)

if nargin == 2
    debug = false;
end


sizes = zeros(1, length(matrix_id) * episodes_per_problem);
dist = zeros(1, length(matrix_id) * episodes_per_problem);
nnz_raw = zeros(1, length(matrix_id) * episodes_per_problem);
nnz_colamd = zeros(1, length(matrix_id) * episodes_per_problem);
nnz_myorder = zeros(1, length(matrix_id) * episodes_per_problem);

for sz = 1:length(matrix_id)
    problem = UFget(matrix_id(sz));

    % Reading the Jacobian
    input_matrix = problem.A;
    
    % If the matrix is square or having less number of rows than twice that
    % of the number of colums (because we are going to split it into two
    % to simulate combining two graphs) then consider only half the
    % variables. We typically need a overdetermined system for LMS.
    if size(input_matrix, 1) < 2 * size(input_matrix, 2)
        input_matrix = input_matrix(:, 1:round(size(input_matrix, 2)/2.5)); % 2.5 is some number greater than 2.
    end
    
    input_matrix_size = size(input_matrix,2);    

    %Splitting the matrix to get two graphs. A subset of variables will
    %later be removed based on random sampling to avoid complete overlap
    %between the two graphs.
    
    % First graph
    input_matrix_A = input_matrix(1:floor(size(input_matrix,1)/2), :);
    % Second graph
    input_matrix_B = input_matrix(ceil(size(input_matrix,1)/2):size(input_matrix,1), :);
    
    % Fixing ill conditioned and disconnected graphs
    [input_matrix_A, original_row_A, original_col_A] = fix_ill_conditioned(input_matrix_A);
    [input_matrix_B, original_row_B, original_col_B] = fix_ill_conditioned(input_matrix_B);
    
    % If all the rows and colums are eliminated as the part of fixing ill
    % conditioned matrix, then SKIP
    if isempty(input_matrix_A) || isempty(input_matrix_B)
        continue;
    end
    
    % Variables saving the sizes of fixed matrices.
    input_matrix_size_A = size(input_matrix_A,2); 
    input_matrix_size_B = size(input_matrix_B,2);
    
    % Finding the parent matrix ordering of graph A.
    input_matrix_order_A = colamd(input_matrix_A);
    % Finding the parent matrix ordering of graph B.
    input_matrix_order_B = colamd(input_matrix_B);
    
    R_A = qr(input_matrix_A);
    R_B = qr(input_matrix_B);

    i = 1;
    while i <= episodes_per_problem
        sample_init_size_A = randi([round(input_matrix_size_A/20) round(input_matrix_size_A/10)], 1);
        sample_init_size_B = randi([round(input_matrix_size_B/20) round(input_matrix_size_B/10)], 1);
        random_pick_vars_A = randsample(input_matrix_size_A, sample_init_size_A)';
        random_pick_vars_B = randsample(input_matrix_size_B, sample_init_size_B)';
        
        total_affected_variables_A = traverse_bayes_tree(R_A, random_pick_vars_A, input_matrix_size_A);
        total_affected_variables_B = traverse_bayes_tree(R_B, random_pick_vars_B, input_matrix_size_B);
        
        trans_total_affected_variables_A = transform_to_higher(total_affected_variables_A, original_col_A);
        trans_total_affected_variables_B = transform_to_higher(total_affected_variables_B, original_col_B);
        
        if isempty(intersect(trans_total_affected_variables_A, trans_total_affected_variables_B))   % intersect in higher space
            continue;
        elseif isequal(trans_total_affected_variables_A, trans_total_affected_variables_B)  % check in higher space
            continue;
        end

        sample_final_size = length(union(total_affected_variables_A, total_affected_variables_B));        
        
        pure_col_indices_A = transform_to_lower(setdiff(transform_to_higher(total_affected_variables_A, original_col_A), ...
            intersect(transform_to_higher(total_affected_variables_A, original_col_A), ...
            transform_to_higher(total_affected_variables_B, original_col_B))), original_col_A); %in higher space and back to lower space
        
        pure_col_indices_B = transform_to_lower(setdiff(transform_to_higher(total_affected_variables_B, original_col_B), ...
            intersect(transform_to_higher(total_affected_variables_A, original_col_A), ...
            transform_to_higher(total_affected_variables_B, original_col_B))), original_col_B); %in higher space and back to lower space
        
        original_common_col_indices = intersect(transform_to_higher(total_affected_variables_A, original_col_A), ...
            transform_to_higher(total_affected_variables_B, original_col_B)); %in higher space and back to lower space in A and B

        lower_common_col_indices_A = transform_to_lower(original_common_col_indices, original_col_A);
        lower_common_col_indices_B = transform_to_lower(original_common_col_indices, original_col_B);
        
                
        sample_matrix = blkdiag(input_matrix_A(:,pure_col_indices_A), input_matrix_B(:,pure_col_indices_B)); 
        for j = 1:length(lower_common_col_indices_A)   % has to be done carefully because j is same for A and B
            sample_matrix = [sample_matrix [input_matrix_A(:,lower_common_col_indices_A(j)); input_matrix_B(:,lower_common_col_indices_B(j))]];
        end

        sample_matrix_colamd_order = colamd(sample_matrix);
        
        myorder_A = relative_ordering(input_matrix_order_A, pure_col_indices_A);    % higher indices instead of pure_col_indices
        myorder_B = relative_ordering(input_matrix_order_B, pure_col_indices_B);    % higher indices instead of pure_col_indices
        myorder_B = myorder_B + length(myorder_A);
        myorder_common = relative_ordering(input_matrix_order_A, lower_common_col_indices_A);   % using common_col_indices of A or B
        myorder_common = myorder_common + length(myorder_B) + length(myorder_A);
        myorder = [myorder_A myorder_B myorder_common];
        
        if debug
            figure;
            plot(diff(sort(myorder)));
            pause;
            clf;
        end
        

        nnz_raw(((sz-1) * episodes_per_problem) + i) = nnz(qr(sample_matrix(:, fliplr(sample_matrix_colamd_order))))/numel(sample_matrix);
        nnz_colamd(((sz-1) * episodes_per_problem) + i) = nnz(qr(sample_matrix(:, sample_matrix_colamd_order)))/numel(sample_matrix);
        nnz_myorder(((sz-1) * episodes_per_problem) + i) = nnz(qr(sample_matrix(:, myorder)))/numel(sample_matrix);

        sizes(((sz-1) * episodes_per_problem) + i) = sample_final_size;
        dist(((sz-1) * episodes_per_problem) + i) = pdist2(sample_matrix_colamd_order, myorder, 'hamming');
        
        i = i + 1;
        
    end    
end

figure;
hold on;
plot(sizes, dist, 'r*');

figure;
hold on;
plot(nnz_raw);
plot(nnz_colamd);
plot(nnz_myorder)

figure;
hold on;
stem(nnz_raw, 'filled', 'Marker', 'd', 'LineWidth', 0.75);
stem(nnz_colamd, 'filled', 'Marker', 'd', 'LineWidth', 0.75);
stem(nnz_myorder, 'filled', 'Marker', 'd', 'LineWidth', 0.75);

hold off;

end

function [fixed_matrix, original_row_indices, original_col_indices] = fix_ill_conditioned(input_matrix)

    original_row_indices = 1:size(input_matrix,1);
    original_col_indices = 1:size(input_matrix,2);

    % Removing row with all zero
    mm = 1;
    while mm <= size(input_matrix, 1)
        if nnz(input_matrix(mm,:)) == 0
            input_matrix(mm,:) = [];
            original_row_indices(mm) = [];
            fprintf('Fixing row %i with all zero\n', mm);
            continue;
        end
        mm = mm + 1;
    end
    
    % Removing column with all zero
    mm = 1;
    while mm <= size(input_matrix, 2)
        if nnz(input_matrix(:,mm)) == 0
            input_matrix(:,mm) = [];
            original_col_indices(mm) = [];
            fprintf('Fixing column %i with all zero\n', mm);
            continue;
        end
        mm = mm + 1;
    end
    
    % Removing disconnected node if any
    mm = 1;
    while mm <= size(input_matrix,1)        
        if nnz(input_matrix(mm,:)) == 1
            oo = find(input_matrix(mm,:) ~= 0);
            if nnz(input_matrix(:,oo)) == 1
                input_matrix(:,oo) = [];
                input_matrix(mm,:) = [];
                original_col_indices(oo) = [];
                original_row_indices(mm) = [];
                fprintf('Fixing row %i and column %i\n', mm, oo);
                continue;
            end
        end
        mm = mm + 1;
    end    
    
    fixed_matrix = input_matrix;
end


function total_affected_variables = traverse_bayes_tree(tree, sample_variables, tree_size)

        k = 1;
        while k <= length(sample_variables)
            row_considered_binary = zeros(1, tree_size);
            row_considered = tree(sample_variables(k),:);
            row_considered_binary(row_considered ~= 0) = 1;
            assert(~any(sample_variables > tree_size));
            for l = sample_variables(k)+1:tree_size
                if row_considered_binary(l) > 0 && ~any(sample_variables == l)
                    sample_variables = [sample_variables l];
                end
            end
            k = k + 1;
        end
        
        total_affected_variables = sample_variables;
end

function myorder = relative_ordering(colamd_order, query_indices)
        query_size = length(query_indices);
        pos_in_input_matrix = zeros(1, query_size);

%     Logs the index in which the value query_indices(j) is present in
%     colamd_order array in pos_in_input_matrix(j)
        for j = 1:query_size
            pos_in_input_matrix(j) = find(colamd_order == query_indices(j));
        end

        myorder = zeros(1, query_size);
        for j = 1:query_size
            [~,ind] = min(pos_in_input_matrix);
            pos_in_input_matrix(ind) = inf;
            myorder(j) = ind;
        end
end

function transformed_array = transform_to_higher(query_indices, map)

    transformed_array = zeros(1, length(query_indices));
    iter = 1;
    for i = query_indices
        transformed_array(iter) = map(i);
        iter = iter + 1;
    end
end

function lower_array = transform_to_lower(query_in_map, map)
    
    lower_array = zeros(1, length(query_in_map));
    for i = 1:length(lower_array)
        lower_array(i) = find(map == query_in_map(i));
    end
end








