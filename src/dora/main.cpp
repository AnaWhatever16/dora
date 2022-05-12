//---------------------------------------------------------------------------------------------------------------------
//  Vertical Engineering Solutions
//---------------------------------------------------------------------------------------------------------------------
// 
//  Copyright 2020 Vertical Engineering Solutions  - All Rights Reserved
// 
//  Unauthorized copying of this file, via any medium is strictly prohibited Proprietary and confidential.
// 
//  All information contained herein is, and remains the property of Vertical Engineering Solutions.  The 
//  intellectual and technical concepts contained herein are proprietary to Vertical Engineering Solutions 
//  and its suppliers and may be covered by UE and Foreign Patents, patents in process, and are protected 
//  by trade secret or copyright law. Dissemination of this information or reproduction of this material is 
//  strictly forbidden unless prior written permission is obtained from Vertical Engineering Solutions.
//
//---------------------------------------------------------------------------------------------------------------------
//
//  Maintainer: acasado@vengineerings.com
//
//---------------------------------------------------------------------------------------------------------------------

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

#include <client/linux/handler/exception_handler.h>

#include <dora/Dora.h>
#include <csignal>
#include <iostream>
#include <string>

dora::Dora *run_dora;

void signalHandler( int _signum ) {
    std::cout << "Captured close signal. Closing politely." << std::endl;
    run_dora->stop();
}

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