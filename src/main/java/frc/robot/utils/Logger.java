package frc.robot.utils;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.sql.Timestamp;

/**
 * This class is used to log values in a CSV file so you can analyze the connection between certain types of values.
 * For example, this class can be used the find how the distance is effected by camera to target vertical angle.
 * Specify what what data type you want to log in the constructor (ex. velocity, acceleration),
 * and log them in the {@link Logger#log(double...)}.
 * After you have inputted all of your data, use {@link Logger#close()} to save the file.
 * The file can be transferred to your computer by FTP and analyzed by excel.
 */
public class Logger {
	private String path;
	private StringBuilder data;

	/**
	 * @param name    the name of the file. Should end with .csv
	 * @param columns the columns name (ex. velocity, acceleration)
	 */
	public Logger(String name, String... columns) {
		if (!name.endsWith(".csv"))
			name = name + ".csv";
		path = Filesystem.getOperatingDirectory() + "/logs/" + getTimeStamp() + '-' + name;
		data = new StringBuilder();
		log(columns);
	}

	/**
	 * @param values the values to be added to the logger
	 */
	public void log(String... values) {
		for (String value : values) {
			data.append(value).append(',');
		}
		data.setCharAt(data.length() - 1, '\n');
	}

	/**
	 * @param values The values that the logger will log. Each value is logged in it's respective column.
	 *               Make sure to enter the values in the same order column names have been entered in the constructor.
	 */
	public void log(double... values) {
		String[] array = new String[values.length];
		for (int i = 0; i < values.length; i++) {
			array[i] = Double.toString(values[i]);
		}
		log(array);
	}

	/**
	 * Saves the file and closes it. Should only be called once.
	 */
	public void close() {
		File file = new File(path);
		File parentFile = file.getParentFile();

		parentFile.mkdirs();

		try {
			file.createNewFile();
			FileWriter writer = new FileWriter(file);
			writer.write(data.toString());
			writer.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private String getTimeStamp() {
		Timestamp timestamp = new Timestamp(System.currentTimeMillis());
		return timestamp.toString();
	}
}