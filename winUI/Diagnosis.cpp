#include "Diagnosis.h"

Diagnosis::Diagnosis() {
    model_weight_path = "..\\libs\\model.pt";
}

Diagnosis::~Diagnosis() {

}

void Diagnosis::setInputData(const std::vector<std::vector<double>> & input_vector_2d) {
    // Flatten the 2D vector into a 1D array to create the tensor
    std::vector<double> flat_vector;
    for (const auto& vec : input_vector_2d) {
        flat_vector.insert(flat_vector.end(), vec.begin(), vec.end());
    }

    // Convert the flattened vector to a torch tensor
    int64_t batch_size = 1;
    int64_t num_vectors = input_vector_2d.size();
    int64_t vector_size = input_vector_2d[0].size();

    input_tensor = torch::from_blob(flat_vector.data(), { batch_size, num_vectors, vector_size }, torch::kDouble).clone();

}

int Diagnosis::inference(void) {

    try {
        model = torch::jit::load(model_weight_path);
    }
    catch (const c10::Error& e) {
        std::cerr << "Error loading the model\n" << e.what() << std::endl;
        return -1;
    }
    model.eval();
    std::cout << "Model loaded successfully" << std::endl;

    // check the input tensor size
    std::vector<int64_t> expected_size = { 1, 6, 10568 }; //expected tensor size
    if (input_tensor.sizes().vec() != expected_size) {
        std::cerr << "Wrong Input tensor size" << std::endl;
        return -1;
    }

    try {
        // Run inference
        output_tensor = model.forward({ input_tensor }).toTensor();

        std::cout << "Inference output: " << output_tensor << std::endl;
    }
    catch (const c10::Error& e) {
        //std::cerr << e << std::endl;
        std::cerr << "Error Inferencing: " << e.what() << std::endl;
        return -1;
    }
    torch::Tensor probabilities = torch::softmax(output_tensor, /*dim=*/1);
    // Find the index of the maximum value (the predicted class)
    torch::Tensor predicted_class = std::get<1>(torch::max(probabilities, /*dim=*/1));
    int class_index = predicted_class.item<int>();
    return class_index;
}