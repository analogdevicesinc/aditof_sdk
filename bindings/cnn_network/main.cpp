#include <iostream>
#include "cppflow/cppflow.h"
#include <exception>
#include <cstdlib>
#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string>

#include <vector>
#include "struct_mapping/struct_mapping.h"

struct Layer
{
    std::string name;
    std::vector<double> weights;
    std::vector<double> bias;
};

std::string getFileNameWeights(std::string fileName)
{
    std::string firstPart = "../json_model/";
    std::string lastPartWeights = "_weights.txt";

    return(firstPart.append(fileName.append(lastPartWeights)));

}
std::string getFileNameBias(std::string fileName)
{
    std::string firstPart = "../json_model/";
    std::string lastPartBias = "_bias.txt";

   return(firstPart.append(fileName.append(lastPartBias)));
}

void readInLayer(std::vector<Layer> network, std::string fileName)
{
    std::string fileNameWeights = getFileNameWeights(fileName);
    std::string fileNameBias = getFileNameBias(fileName);

    std::ifstream myFileWeights; // creates stream myFile
    std::ifstream myFileBias;    // creates stream myFile

    myFileWeights.open(fileNameWeights);                   // opens .txt file
    myFileBias.open(fileNameBias);                         // opens .txt file
    if (!myFileWeights.is_open() || !myFileBias.is_open()) // check file is open, quit if not
    {
        std::cerr << "failed to open file\n";
        return;
    }

    std::vector<double> weights; // vector to store the numerical values in
    std::vector<double> bias;    // vector to store the numerical values in

    double number = 0;
    while (myFileWeights >> number)
    {
        weights.push_back(number);
    }
    while (myFileBias >> number)
    {
        bias.push_back(number);
    }

    Layer layer;
    layer.name=fileName;
    layer.weights=weights;
    layer.bias=bias;

    network.push_back(layer);

    std::cout << "########\nWeights: " << weights.size() << "\nBiases: " << bias.size() << std::endl;
}

int main()
{
    std::vector<Layer> Network;
    readInLayer(Network, "layer_0");
    readInLayer(Network, "layer_1");
    readInLayer(Network, "layer_2");
    readInLayer(Network, "layer_3");

    return 0;
    /*
        // Load the model
        cppflow::model model("../tensorflow/saved_model/my_model-20220405-111254");

        float test[300] = {1.f};
        std::vector<float> test2(std::begin(test), std::end(test));
        std::vector<int64_t> size;
        size.push_back(300);
        auto input = cppflow::tensor(test2, size);
        // Load an image
        //auto input = cppflow::decode_jpeg(cppflow::read_file(std::string("image.jpg")));


        // Cast it to float, normalize to range [0, 1], and add batch_dimension
        auto input2 = cppflow::cast(input, TF_UINT8, TF_FLOAT);
        //input = input / 255.f;
        input2 = cppflow::expand_dims(input2, 0);

        // Run
        auto output = model(input2);

        // Show the predicted class
        std::cout << cppflow::arg_max(output, 1) << std::endl;

    */
}
