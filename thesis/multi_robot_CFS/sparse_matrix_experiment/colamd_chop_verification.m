function [] = colamd_chop_verification(episodes_per_problem, matrix_id)


sizes = zeros(1, length(matrix_id) * episodes_per_problem);
dist = zeros(1, length(matrix_id) * episodes_per_problem);
nnz_raw = zeros(1, length(matrix_id) * episodes_per_problem);
nnz_colamd = zeros(1, length(matrix_id) * episodes_per_problem);
nnz_myorder = zeros(1, length(matrix_id) * episodes_per_problem);

for sz = 1:length(matrix_id)
    problem = UFget(matrix_id(sz));

    input_matrix = problem.A;
    input_matrix_size = size(input_matrix,2);
    input_matrix_order = colamd(input_matrix);


    for i = 1:episodes_per_problem
        sample_matrix_size = randi([round(input_matrix_size/2) input_matrix_size], 1);
        sample_matrix_col_indices = randsample(input_matrix_size, sample_matrix_size);
        sample_matrix = input_matrix(:, sample_matrix_col_indices);
        sample_matrix_colamd_order = colamd(sample_matrix);

        pos_in_input_matrix = zeros(1, sample_matrix_size);
        for j = 1:sample_matrix_size
    %         pos_in_input_matrix(j) = input_matrix_order(sample_matrix_col_indices(j));
            pos_in_input_matrix(j) = find(input_matrix_order == sample_matrix_col_indices(j));
        end

%         [~, mlab_order] = sort(pos_in_input_matrix);
        sample_matrix_myorder = zeros(1, sample_matrix_size);
%         sample_matrix_myordera = zeros(1, sample_matrix_size);

        for j = 1:sample_matrix_size
            [~,ind] = min(pos_in_input_matrix);
            pos_in_input_matrix(ind) = inf;
            sample_matrix_myorder(j) = ind;
    %         sample_matrix_myordera(j) = find(sample_matrix_col_indices(ind) == sample_matrix_col_indices);
        end

    %     assert(isequal(sample_matrix_myordera, sample_matrix_myorder));


        nnz_raw(((sz-1) * episodes_per_problem) + i) = nnz(qr(sample_matrix))/numel(sample_matrix);
        nnz_colamd(((sz-1) * episodes_per_problem) + i) = nnz(qr(sample_matrix(:, sample_matrix_colamd_order)))/numel(sample_matrix);
        nnz_myorder(((sz-1) * episodes_per_problem) + i) = nnz(qr(sample_matrix(:, sample_matrix_myorder)))/numel(sample_matrix);

        sizes(((sz-1) * episodes_per_problem) + i) = sample_matrix_size;
        dist(((sz-1) * episodes_per_problem) + i) = pdist2(sample_matrix_colamd_order, sample_matrix_myorder, 'hamming');
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



