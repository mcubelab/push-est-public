//
// Created by mcube on 1/13/17.
//

#ifndef ISAMSHAPEPOSE_POINT1D_H
#define ISAMSHAPEPOSE_POINT1D_H


class Point1d {
    friend std::ostream& operator<<(std::ostream& out, const Point1d& p) {
      p.write(out);
      return out;
    }

    double _x;

public:
    static const int dim = 1;
    static const int size = 1;
    static const char* name() {
      return "Point1d";
    }
    Point1d() : _x(0) {}
    Point1d(double x) : _x(x) {}

    double x() const {return _x;}

    void set(double x) {
      _x = x;
    }

    void set(const VectorXd& x) {
      _x = x[0];
    }

    Point1d exmap(const Eigen::Vector1d& delta) const {
      Point1d res = *this;
      res._x += delta(0);
      return res;
    }

    void write(std::ostream &out) const {
      out << "(" << _x << ")";
    }

    Eigen::VectorXb is_angle() const {
      return Eigen::VectorXb::Zero(size);
    }

    Eigen::Vector1d vector() const {
      Eigen::Vector1d v(_x);
      return v;
    }
};

#endif //ISAMSHAPEPOSE_POINT1D_H
