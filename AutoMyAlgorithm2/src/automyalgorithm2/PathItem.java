/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package automyalgorithm2;

import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author sev_user
 */
public class PathItem {
    List<Integer> path;

    public PathItem() {
        path = new ArrayList<>();
    }

    
    public PathItem(List<Integer> path) {
        this.path = path;
    }

    public List<Integer> getPath() {
        return path;
    }

    public void setPath(List<Integer> path) {
        this.path = path;
    }
    
    public int size() {
        if (path != null) {
            return path.size();
        }
        return 0;
    }
    
    public void addElement(int element) {
        if (path != null) {
            path.add(element);
        }
    }
    
    public int getElement(int pos) {
        if (path != null && pos <path.size()) {
            return path.get(pos);
        }
        return -1;
    }
    
    public static boolean compareSame (PathItem path1 , PathItem path2) {
        int count =0;
        if (path1.size() == path2.size()) {
            for (int i = 0; i< path1.size(); i++) {
                if (path1.getElement(i) != path2.getElement(i)) {
                    return false;
                } else {
                    count ++;
                }
            }
            if (count == path1.size()) return true;
            
        }
        return false;
    }
    
    public String getString() {
        if (path == null) return "null";
        if (path.size() == 1) return " "+path.get(0);
        StringBuilder sb = new StringBuilder();
        for (int i =0; i< path.size()-1;i++) {
            sb.append(path.get(i)+"-");
        }
        sb.append(path.get(path.size()-1)+"");
        return sb.toString();
    }
    
    
}
