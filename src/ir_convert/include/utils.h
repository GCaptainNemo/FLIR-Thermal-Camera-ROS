#ifndef UTILS_H_
#define UTILS_H_

#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>

class converter_16_8
{
  enum
  {
    histminmembersperbucket = 10,
  };
private:
  double max_;
  double min_;
  static converter_16_8* inst_;
  bool firstframe_;
  // boost::shared_ptr<cv::Retina> retina_;
public:
  converter_16_8();
  ~converter_16_8();
  static converter_16_8& Instance()
  {
    if (!inst_)
    {
      inst_ = new converter_16_8;
    }
    return *inst_;
  }
  double getMax();
  double getMin();
  void convert_to8bit(const cv::Mat& img16, cv::Mat& img8, bool doTempConversion);
  void toneMapping(const cv::Mat& img16, cv::Mat& img8);
};

struct color
{
  unsigned char rgbBlue;
  unsigned char rgbGreen;
  unsigned char rgbRed;
  color()
  {
    rgbBlue = rgbGreen = rgbRed = 0;
  }
};

struct palette
{
  enum palettetypes{
    Linear_red_palettes, GammaLog_red_palettes, Inversion_red_palette, Linear_palettes, GammaLog_palettes,
    Inversion_palette, False_color_palette1, False_color_palette2, False_color_palette3, False_color_palette4
  };
  color colors[256];
};

palette GetPalette(palette::palettetypes pal);
palette GetPalette(const std::string & pal);

void convertFalseColor(const cv::Mat& srcmat, cv::Mat& dstmat, const palette &pal, bool drawlegend = false, double mintemp = 0, double maxtemp = 0);

#endif /* UTILS_H_ */
