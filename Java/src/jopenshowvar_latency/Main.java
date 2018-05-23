package jopenshowvar_latency;

public class Main {	
    public static void main(String[] args) {
	// Default config
	int NUM_THREAD = 5;
	int num_tests = 5000;
	boolean e6axis = true;
	// Parse command line input
	for (int i = 0; i < args.length; i++){
	    switch (args[i].charAt(0)) {
	    case '-':
		if (args[i].length < 2) {
		    throw new IllegalArgumentException("Not a valid argument " +args[i]);
		}
		switch (args[i].charAt(1)) {
		case 'h':
		    System.out.println("jopenshowvar_latency usage");
		    System.out.println("-h \t print this help");
		    System.out.println("-t # \t set number of threads, default=1");
		    System.out.println("-r # \t set number of tests, default=100");
		    System.out.println("-e # \t true is e6axis, false is int, default=0");
		    System.exit();
		case 't':
		    NUM_THREAD = Integer.parseInt(args[i+1]);
		    i++;
		case 'r':
		    num_tests = Integer.parseInt(args[i+1]);
		    i++;
		case 'e':
		    e6axis = Boolean.parseBoolean(args[i+1]);
		    i++;
		}
	    }
	}
	System.out.println("Thread, #Threads, Test type, e6axis, Test #, Time (s)");
	
	
	Thread threads[] = new Thread[NUM_THREAD];;
	
	for (int i=0; i < NUM_THREAD; i++) {
	    threads[i] = new Thread(new Worker(i, NUM_THREAD, e6axis, num_tests), String.valueOf(i));
	    threads[i].start();
	}
	
	try {
	    for (int i=0; i < NUM_THREAD; i++) {
		threads[i].join();
	    }
	} catch (InterruptedException e) {
	    // TODO Auto-generated catch block
	    e.printStackTrace();
	}
    }
}
