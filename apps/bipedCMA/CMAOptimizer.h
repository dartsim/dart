/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef APP_BALANCE_CMAOPTIMIZER_H_
#define APP_BALANCE_CMAOPTIMIZER_H_

class MyWindow; // manages simulation through this class

class CMAOptimizer {
public:
  CMAOptimizer();
  virtual ~CMAOptimizer();

  bool solve();

  void setMyWindow(MyWindow* _window);

protected:
  MyWindow* mWindow;
  
}; // class CMAOptimizer


#endif // #ifndef APP_BALANCE_CMAOPTIMIZER_H_

