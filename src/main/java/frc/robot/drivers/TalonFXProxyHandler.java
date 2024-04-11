// Copyright (c) 2024 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;

// Please don't try to refactor this to generalize
// to other classes that have a getConfigurator method.
// Save your time and just don't do it. I can't get the types to work
public class TalonFXProxyHandler implements InvocationHandler {
  private Object target;

  public TalonFXProxyHandler(Object target) {
    this.target = target;
  }

  @Override
  public Object invoke(Object proxy, Method method, Object[] args) throws Throwable {
    if (method.getName().equals("getConfigurator")) {
      TalonFXConfigurator configurator = (TalonFXConfigurator) method.invoke(target, args);
      return new WBConfigurator<>((TalonFXConfiguration x) -> configurator.apply(x));
    }
    return method.invoke(target, args);
  }
}
