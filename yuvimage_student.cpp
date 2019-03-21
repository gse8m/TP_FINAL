//
// header-start
//////////////////////////////////////////////////////////////////////////////////
//
// \file      yuvimage.cpp
//
// \brief     This file belongs to the C++ tutorial project
//
// \author    Bernard
//
// \copyright Copyright 2019
//
//////////////////////////////////////////////////////////////////////////////////
// header-end
//
#include "yuvimage_student.h"
#include <QString>
#include <QDebug>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <memory>
//#include <Eigen/Dense>
#include "gse4_util.h"
#include <chrono>


// Convert a single yuv point to rgb color
// based on BT.709 formula
// Y = 0.2126*R + 0.7152*G + 0.0722*B
//
static
QRgb convert_yuv_to_rgb(uint8_t y, uint8_t u, uint8_t v) {
    static GSE4::Clamp<double> clamp_to_rgb(0., 255.);
    double r = y + 0*u       + 1.28033*v;
    double g = y - 0.21482*u - 0.38059*v;
    double b = y + 2.12798*u +       0*v;
    return qRgb(clamp_to_rgb(r), clamp_to_rgb(g), clamp_to_rgb(b));
}

// Convert raw yuv data to rgb image
// process image pixel by pixel
// calling the conversion yuv to rgb each time
// The raw yuv data is in 4:2:0 format
// i.e a single chroma value (u or v) is applicable to 4 luma values.
// We haven't implemented the filtering for upscaling described in
// https://msdn.microsoft.com/en-us/library/windows/desktop/dd206750(v=vs.85).aspx
//

int YuvImage::trans(int i) {
    int rang = (i/width_)/2;
    int colone = (i%width_)/2;
    return colone+rang*(width_/2);
}

void YuvImage::yuv_to_rgb(size_t groupe_idx) {
    int j;
    int i = 0;

    r_ = new double [width_*height_];
    g_ = new double [width_*height_];
    b_ = new double [width_*height_];

   for(j=0 ; j < width_*height_ ; ++j) {
        i=YuvImage::trans(j);
        r_[j] = 1.1643836*y_raw_[j] +         0*u_raw_[i] + 1.8765140*v_raw_[i] - 268.2065015;
        g_[j] = 1.1643836*y_raw_[j] - 0.2132486*u_raw_[i] - 0.5578116*v_raw_[i] + 82.8546329;
        b_[j] = 1.1643836*y_raw_[j] + 2.1124018*u_raw_[i] -         0*v_raw_[i] - 289.0175656;
    }

}

// Assist function for YUV to RGB image creation
// allocate the memory buffers
// and read raw yuv from the ifstream into the buffers
//
void YuvImage::load_from_stream(std::ifstream &yuv_strm) {
    auto ysize = width_*height_;
    auto uv_size = ysize >> 2;

    y_raw_ = new uint8_t [ysize];
    u_raw_ = new uint8_t [uv_size];
    v_raw_ = new uint8_t [uv_size];

    yuv_strm.read(reinterpret_cast<char*>(y_raw_), ysize);
    yuv_strm.read(reinterpret_cast<char*>(u_raw_), uv_size);
    yuv_strm.read(reinterpret_cast<char*>(v_raw_), uv_size);

}

// Mesure du temps
void YuvImage::time() {
    size_t i=0;
    auto time_start = std::chrono::high_resolution_clock::now();
    //12 fois
  //  for (int j=0; i<12; i++) {
        YuvImage::yuv_to_rgb(i);
  //  }
    auto time_end = std::chrono::high_resolution_clock::now();

    auto elapsed_time_us = std::chrono::duration_cast<std::chrono::microseconds> (time_end - time_start);
    long long int elapsed_time_us_eff = elapsed_time_us.count();
    std::string s = std::to_string(elapsed_time_us_eff);
    std::cout << "INFO: image calculed in " << s << std::endl;
    // Séparer le temps par virgule
    uint id = 0;
    const auto end_ = s.end();
    const auto size_ = s.size(); // pour éviter le changement de pointeur
    for (id = 3; id < size_; id += 3) {
        s.insert (end_ - id, ',');
    }

    //Imprimer sur le terminal
    std::cout << "INFO: image calculed in " << s << " us" << std::endl;
}


//! \brief
//! Constructor for the YuvImage
//! Note that the default QImage
//! will not be used except if a throw.
//! We shall use QImage::swap at the end
//
static const int default_width = 400;
static const int default_height = 400;

YuvImage::YuvImage(const std::string &file_name) :
  QImage(default_width, default_height, QImage::Format_RGB32),
  width_{default_width},
  height_{default_height} {
  static GSE4::Clamp<double> clamp_to_rgb(0., 255.);

  // we find the width and height of the file
  // based on the size of the file


  // student version
  // all try catch block removed
  std::ifstream yuv_strm(file_name, std::ios::in | std::ios::binary);

  // position stream pointer at the end to read the file size...
  yuv_strm.seekg(0, std::ios::end);
  auto filesize =  yuv_strm.tellg();

  // .. and don't forget to put it back to the beg so that we can read
  yuv_strm.seekg(0, std::ios::beg);

  // a few lines of code needed here
  // to compute  width and height from the size of the file

  auto ysize = (filesize * 2) / 3;
  if (((ysize * 3) / 2) != filesize && (((ysize * 3) / 2) % 6) == 0) {
    // more code is needed here
    throw wrong_size();
    qDebug("Wrong size");
  }
    //auto uv_size = ysize >> 2;
    switch(ysize) {
     case(28000*4762): width_ = 28000; height_ = 4762; break;
     case(10000*4762): width_ = 10000; height_ = 4762; break;
     case(3840*2160) : width_ =  3840; height_ = 2160; break;
     case(1920*1080) : width_ =  1920; height_ = 1080; break;
     case(1024*768)  : width_ =  1024; height_ =  768; break;
     case(832*480)   : width_ =   832; height_ =  480; break;
     case(352*288)   : width_ =   352; height_ =  288; break;
     default: throw wrong_size();
   }
    size_t i=0;
    YuvImage::load_from_stream(yuv_strm);
    YuvImage::yuv_to_rgb(i);
    YuvImage::time();
  // create local image of given size
  // and swap with current image of the class
  QImage main_image(width_, height_, QImage::Format_RGB32);
  swap(main_image);

  for (int yp = 0; yp < height_; ++yp) {
    for (int xp = 0; xp < width_; ++xp) {
      setPixel(xp, yp, qRgb(clamp_to_rgb(r_[xp+yp*width_]), clamp_to_rgb(g_[xp+yp*width_]), clamp_to_rgb(b_[xp+yp*width_])));
      }
  }

  delete [] y_raw_;
  delete [] u_raw_;
  delete [] v_raw_;

  delete [] r_;
  delete [] g_;
  delete [] b_;

}

