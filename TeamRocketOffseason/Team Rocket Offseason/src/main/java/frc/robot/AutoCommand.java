package frc.robot;

import java.util.ArrayList;

public class AutoCommand {
    private int id;
    private ArrayList<String> otherPertinentData = new ArrayList<>();

    public AutoCommand(int x, ArrayList<String> data) {
        id = x;
        otherPertinentData = data;
    }

    public void run() {
        System.out.println("test failed succesfully!");
    }
}