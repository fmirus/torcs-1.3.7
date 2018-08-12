#ifndef GRAPH1010
#define GRAPH1010
#include <deque>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

class Plot {
  public:
    Plot() = default;
    Plot(int maxSize = 100, bool persist = true) {
        std::cout << "Initialition of plot with max " << maxSize << " elements ";
        _maxSize = maxSize;
        _persist = persist;
        if (_persist)
            _gp = popen("gnuplot -persist", "w");
        else
            _gp = popen("gnuplot", "w");

        if (_gp == NULL) {
            std::cout << "failed" << std::endl;
        } else {
            std::cout << "successfull" << std::endl;
        }
        update(1);
    }

    void update(float y) {
        if (_x.empty()) {
            update(0, y);
        } else {
            update(++_x.back(), y);
        }
    }

    void update(float x, float y) {
        _x.push_back(x);
        _y.push_back(y);
        if (_x.size() > _maxSize) {
            _x.pop_front();
        }
        if (_y.size() > _maxSize) {
            _y.pop_front();
        }
        std::cout << "Update, size now " << _x.size() << std::endl;
        plot_data();
    }

    void setData(std::deque<float> inY, bool checkSize = true) {
        _y = inY;
        if (checkSize) {
            while (_y.size() > _maxSize) {
                _y.pop_front();
            }
        }
        if (_x.size() != _y.size()) {
            _x = std::deque<float>();
            for (int i = 0; i < _x.size(); ++i) {
                _x.push_back(i);
            }
        }
    }

    void plot_data(const char *style = "points", const char *title = "Data") {
        // fprintf(_gp, "set title '%s' \n", title);
        // fprintf(_gp, "plot '-' w %s \n", style);
        // for (int k = 0; k < _x.size(); k++) {
        // fprintf(_gp, "%f %f \n", _x[k], _y[k]);
        // }
        // fprintf(_gp, "e\n");
        // fflush(_gp);
    }

    ~Plot() { pclose(_gp); }

  private:
    FILE *_gp;
    bool _persist;
    std::deque<float> _x, _y;
    unsigned int _maxSize;
};

#endif
