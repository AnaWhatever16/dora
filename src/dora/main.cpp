// MIT License

// Copyright (c) [year] [fullname]

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

///////////////////////////////////////////////////////////
//                                                       //
//       ///////     //////    ///////          /////    //      
//      //     //   //    //   //    //       //   //    //  
//     //      //  //      //  //    //      //    //    //  
//    //      //   //      //  ///////      /////////    //   
//   //      //     //    //   //    //    //      //    //   
//  ////////         /////     //     //  //       //    //   
//                                                       //
//                                                       //
//  WIP NAME                                             //
//                                                       //
///////////////////////////////////////////////////////////

#include <dora/Dora.h>
#include <csignal>
#include <iostream>
#include <string>

dora::Dora *run_dora;

int main(int _argc, char**_argv){
    std::string path2Config = _argv[1];

    run_dora = new dora::Dora();
    if(!run_dora->init(path2Config)){
        std::cout << "Error configuring Dora. Cannot start." << std::endl;
        delete run_dora;   // Call destructor properly
        return -1;
    }

    try{
        run_dora->start();
    }catch(std::exception &_e){
        std::cout << _e.what() << std::endl;
    }

    std::cout << "Deleting Dora" << std::endl;
    delete run_dora;
    return 0;
}