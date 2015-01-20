import java.io.*;

public class IdeHelper {
//./lsz -vv --escape --binary Readme.txt  <> /dev/ttyACM0 1>&0
	private static String lszPath = "./lsz";
	private static String lszArgs = " --binary --escape ";
	private static String lszCommand = lszArgs + "-C 3 -c ";
	private static String lszUpload = lszArgs;
	private static Integer MAGIC = -488273;
	private static String execOutput;
	private static String execError;
	
	private static void print(String s) {
		System.out.println(s);
	}

	/* function that executes process 
	@TODO "/bin/sh" not present on Windows and possibly MAC OSx, fix */
	private static int exec(String s)  {
		String output = "";
		String error = "";
		Integer exitcode = MAGIC;

		System.out.println("executing: " + s);

		try {
			Process p = Runtime.getRuntime().exec(new String[] {"/bin/sh", "-c", s});
			p.waitFor();
			exitcode = p.exitValue();

			BufferedReader stdin = new BufferedReader(
				new InputStreamReader(p.getInputStream()));
			BufferedReader stderr = new BufferedReader(
				new InputStreamReader(p.getErrorStream()));

			execOutput = "";
			while ((output = stdin.readLine()) != null) {
				execOutput += output;
			}
	
			execError = "";
			while ((error = stderr.readLine()) != null) {
				execError += error;
			}
		} catch (IOException e) {
			System.out.println("Exception: ");
			e.printStackTrace();
			System.exit(-1);
		} catch (InterruptedException e) {
			System.out.println("Exception: ");
			e.printStackTrace();
			System.exit(-1);
		}
	
		return exitcode;
	}

	private static Integer execLsz(String s) {
		File f = new File(lszPath);
		if(!f.exists()) {
			print(lszPath + " does not exist!");
			System.exit(-1);
		}
		return exec(lszPath + s + getDevice());
	}
	public static Integer execLszCommand(String s) {
		Integer ret = execLsz(lszCommand + "\"" + s  + "\"");
		if(execError.lastIndexOf("Transfer ") >= 0) {
			execError = execError.substring(0, execError.lastIndexOf("Transfer "));
		}
		return ret;
	}	
	public static Integer getFlashVersion() {
		Integer ret = execLszCommand("cat /sys/firmware/board_data/flash_version");
		if(ret == 0) {
			System.out.println("Found Flash.bin version: " + execError);
		} else {
			System.out.println("Flash.bin version not found");
		}
		return ret;
	}
	
	/* @TODO find out what to do on Windows/MAC OSx*/
	private static String getDevice() {
		return " <> /dev/ttyACM0";
	}

	public static Integer uploadFile(String path) {
		return execLsz(lszUpload + path + getDevice());
	}

	public static void main(String[] args) {
		Integer ret = getFlashVersion();
		//System.out.println(ret.toString() + " " + execError);
		uploadFile("Readme.txt");
	}

}
