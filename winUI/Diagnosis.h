#pragma once

#include <torch/script.h>
#include <torch/torch.h>
#include <memory>
#include <ATen/ATen.h>
#include <iostream>
#include <vector>

class Diagnosis
{
public:
	Diagnosis();
	~Diagnosis();
	int inference(void);
	void setInputData(const std::vector<std::vector<double>> &);
	

private:
	std::string model_weight_path;
	std::vector<std::vector<double>> joint_err;
	at::Tensor input_tensor;
	at::Tensor output_tensor;

	torch::jit::script::Module model;
};

