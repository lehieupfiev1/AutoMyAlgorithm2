/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package automyalgorithm2;

/**
 *
 * @author sev_user
 */
public class FloatPointItem {
    float x;
    float y;
    
    public FloatPointItem() {
        
    }

    public FloatPointItem(float x, float y) {
        this.x = x;
        this.y = y;
    }

    public float getX() {
        return x;
    }

    public void setX(float x) {
        this.x = x;
    }

    public float getY() {
        return y;
    }

    public void setY(float y) {
        this.y = y;
    }
    
    public void setXY(float x,float y) {
        this.x = x;
        this.y = y;
    }
    
}
