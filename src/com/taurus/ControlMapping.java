/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package com.taurus;

/**
 * This class abstracts control mappings slightly.
 *
 * @author Tanner Danzey < arkaniad AT gmail DOT com >
 */
public class ControlMapping {

    public static final int grabberArmDown = 2;
    public static final int grabberArmUp = 3;
    public static final int I2CGet = 10;
    public static final int I2CSet = 11;

    public static final int grabberMotorForward = 3;
    public static final int grabberMotorReverse = 2;

    public static final int latchLeft = 8;
    public static final int latchRight = 8;

    public static final int releaseLeft = 9;
    public static final int releaseRight = 9;

    public static final int fireLeft = 1;
    public static final int fireRight = 1;

    public static final int camUp = 5;
    public static final int camDown = 4;
    
    public static final int driveFacing = 7;

}
