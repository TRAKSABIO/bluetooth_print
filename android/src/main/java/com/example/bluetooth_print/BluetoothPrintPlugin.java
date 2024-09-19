package com.example.bluetooth_print;

import android.Manifest;
import android.app.Activity;
import android.app.Application;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothSocket;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.util.Log;
import android.widget.Toast;

import androidx.annotation.RequiresApi;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.example.tscdll.TSCActivity;
import com.google.firebase.crashlytics.buildtools.reloc.com.google.common.base.Joiner;
import com.gprinter.command.FactoryCommand;
import com.gprinter.command.LabelCommand;

import io.flutter.embedding.engine.plugins.FlutterPlugin;
import io.flutter.embedding.engine.plugins.activity.ActivityAware;
import io.flutter.embedding.engine.plugins.activity.ActivityPluginBinding;
import io.flutter.plugin.common.*;
import io.flutter.plugin.common.EventChannel.EventSink;
import io.flutter.plugin.common.EventChannel.StreamHandler;
import io.flutter.plugin.common.MethodChannel.MethodCallHandler;
import io.flutter.plugin.common.MethodChannel.Result;
import io.flutter.plugin.common.PluginRegistry.Registrar;
import io.flutter.plugin.common.PluginRegistry.RequestPermissionsResultListener;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.Arrays;
import java.util.StringJoiner;

/**
 * BluetoothPrintPlugin
 * @author thon
 */
public class BluetoothPrintPlugin implements FlutterPlugin, ActivityAware, MethodCallHandler, RequestPermissionsResultListener {
  private static final String TAG = "BluetoothPrintPlugin";
  private Object initializationLock = new Object();
  private Context context;
  private ThreadPool threadPool;
  private String curMacAddress;

  private static final String NAMESPACE = "bluetooth_print";
  private MethodChannel channel;
  private EventChannel stateChannel;
  private BluetoothManager mBluetoothManager;
  private BluetoothAdapter mBluetoothAdapter;

  private FlutterPluginBinding pluginBinding;
  private ActivityPluginBinding activityBinding;
  private Application application;
  private Activity activity;

  private MethodCall pendingCall;
  private Result pendingResult;
  private static final int REQUEST_FINE_LOCATION_PERMISSIONS = 1452;


  ///TSCActivity
  private BluetoothSocket btSocket = null;
  private OutputStream OutStream = null;
  private InputStream InStream = null;
  public boolean IsConnected = false;
  private static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
  TSCActivity TscDll = new TSCActivity();//BT
  ///TSCActivity

  private static String[] PERMISSIONS_LOCATION = {
          Manifest.permission.BLUETOOTH,
          Manifest.permission.BLUETOOTH_ADMIN,
          Manifest.permission.BLUETOOTH_CONNECT,
          Manifest.permission.BLUETOOTH_SCAN,
          Manifest.permission.ACCESS_FINE_LOCATION
  };

  public static void registerWith(Registrar registrar) {
    final BluetoothPrintPlugin instance = new BluetoothPrintPlugin();

    Activity activity = registrar.activity();
    Application application = null;
    if (registrar.context() != null) {
      application = (Application) (registrar.context().getApplicationContext());
    }
    instance.setup(registrar.messenger(), application, activity, registrar, null);
  }

  public BluetoothPrintPlugin(){
  }


  @Override
  public void onAttachedToEngine(FlutterPluginBinding binding) {
    pluginBinding = binding;
  }

  @Override
  public void onDetachedFromEngine(FlutterPluginBinding binding) {
    pluginBinding = null;
  }

  @Override
  public void onAttachedToActivity(ActivityPluginBinding binding) {
    activityBinding = binding;
    setup(
            pluginBinding.getBinaryMessenger(),
            (Application) pluginBinding.getApplicationContext(),
            activityBinding.getActivity(),
            null,
            activityBinding);
  }

  @Override
  public void onDetachedFromActivity() {
    tearDown();
  }

  @Override
  public void onDetachedFromActivityForConfigChanges() {
    onDetachedFromActivity();
  }

  @Override
  public void onReattachedToActivityForConfigChanges(ActivityPluginBinding binding) {
    onAttachedToActivity(binding);
  }

  private void setup(
          final BinaryMessenger messenger,
          final Application application,
          final Activity activity,
          final PluginRegistry.Registrar registrar,
          final ActivityPluginBinding activityBinding) {
    synchronized (initializationLock) {
      Log.i(TAG, "setup");
      this.activity = activity;
      this.application = application;
      this.context = application;
      channel = new MethodChannel(messenger, NAMESPACE + "/methods");
      channel.setMethodCallHandler(this);
      stateChannel = new EventChannel(messenger, NAMESPACE + "/state");
      stateChannel.setStreamHandler(stateHandler);
      mBluetoothManager = (BluetoothManager) application.getSystemService(Context.BLUETOOTH_SERVICE);
      mBluetoothAdapter = mBluetoothManager.getAdapter();
      if (registrar != null) {
        // V1 embedding setup for activity listeners.
        registrar.addRequestPermissionsResultListener(this);
      } else {
        // V2 embedding setup for activity listeners.
        activityBinding.addRequestPermissionsResultListener(this);
      }
    }
  }

  private void tearDown() {
    Log.i(TAG, "teardown");
    context = null;
    activityBinding.removeRequestPermissionsResultListener(this);
    activityBinding = null;
    channel.setMethodCallHandler(null);
    channel = null;
    stateChannel.setStreamHandler(null);
    stateChannel = null;
    mBluetoothAdapter = null;
    mBluetoothManager = null;
    application = null;
  }


  @Override
  public void onMethodCall(MethodCall call, Result result) {
    if (mBluetoothAdapter == null && !"isAvailable".equals(call.method)) {
      result.error("bluetooth_unavailable", "the device does not have bluetooth", null);
      return;
    }

    switch (call.method){
      case "state":
        state(result);
        break;
      case "isAvailable":
        result.success(mBluetoothAdapter != null);
        break;
      case "isOn":
        result.success(mBluetoothAdapter.isEnabled());
        break;
      case "isConnected":
        result.success(threadPool != null);
        break;
      case "startScan":
      {
        if (ContextCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
          ActivityCompat.requestPermissions(activityBinding.getActivity(), PERMISSIONS_LOCATION, REQUEST_FINE_LOCATION_PERMISSIONS);
          pendingCall = call;
          pendingResult = result;
          break;
        }

        startScan(call, result);
        break;
      }
      case "stopScan":
        stopScan();
        result.success(null);
        break;
      case "connect":
        connect(call, result);
        break;
      case "disconnect":
        result.success(disconnect());
        break;
      case "destroy":
        result.success(destroy());
        break;
      case "print":
      case "printReceipt":
      case "printLabel":
        print(call, result);
        break;
      case "printTest":
        printTest(result);
        break;
      default:
        result.notImplemented();
        break;
    }

  }

  private void getDevices(Result result){
    List<Map<String, Object>> devices = new ArrayList<>();
    for (BluetoothDevice device : mBluetoothAdapter.getBondedDevices()) {
      Map<String, Object> ret = new HashMap<>();
      ret.put("address", device.getAddress());
      ret.put("name", device.getName());
      ret.put("type", device.getType());
      devices.add(ret);
    }

    result.success(devices);
  }

  /**
   * 获取状态
   */
  private void state(Result result){
    try {
      switch(mBluetoothAdapter.getState()) {
        case BluetoothAdapter.STATE_OFF:
          result.success(BluetoothAdapter.STATE_OFF);
          break;
        case BluetoothAdapter.STATE_ON:
          result.success(BluetoothAdapter.STATE_ON);
          break;
        case BluetoothAdapter.STATE_TURNING_OFF:
          result.success(BluetoothAdapter.STATE_TURNING_OFF);
          break;
        case BluetoothAdapter.STATE_TURNING_ON:
          result.success(BluetoothAdapter.STATE_TURNING_ON);
          break;
        default:
          result.success(0);
          break;
      }
    } catch (SecurityException e) {
      result.error("invalid_argument", "argument 'address' not found", null);
    }

  }


  private void startScan(MethodCall call, Result result) {
    Log.d(TAG,"start scan ");

    try {
      startScan();
      result.success(null);
    } catch (Exception e) {
      result.error("startScan", e.getMessage(), e);
    }
  }

  private void invokeMethodUIThread(final String name, final BluetoothDevice device)
  {
    final Map<String, Object> ret = new HashMap<>();
    ret.put("address", device.getAddress());
    ret.put("name", device.getName());
    ret.put("type", device.getType());

    activity.runOnUiThread(
            new Runnable() {
              @Override
              public void run() {
                channel.invokeMethod(name, ret);
              }
            });
  }

  private ScanCallback mScanCallback = new ScanCallback() {
    @Override
    public void onScanResult(int callbackType, ScanResult result) {
      BluetoothDevice device = result.getDevice();
      if(device != null && device.getName() != null){
        invokeMethodUIThread("ScanResult", device);
      }
    }
  };

  private void startScan() throws IllegalStateException {
    BluetoothLeScanner scanner = mBluetoothAdapter.getBluetoothLeScanner();
    if(scanner == null) {
      throw new IllegalStateException("getBluetoothLeScanner() is null. Is the Adapter on?");
    }

    // 0:lowPower 1:balanced 2:lowLatency -1:opportunistic
    ScanSettings settings = new ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build();
    scanner.startScan(null, settings, mScanCallback);
  }

  private void stopScan() {
    BluetoothLeScanner scanner = mBluetoothAdapter.getBluetoothLeScanner();
    if(scanner != null) {
      scanner.stopScan(mScanCallback);
    }
  }

  /**
   * 连接
   */
  private void connect(MethodCall call, Result result){
    Map<String, Object> args = call.arguments();
    if (args !=null && args.containsKey("address")) {
      final String address = (String) args.get("address");
      this.curMacAddress = address;

      Log.e(TAG, "##### address -> " + address.toString());

      disconnect();

      new DeviceConnFactoryManager.Build()
              //设置连接方式
              .setConnMethod(DeviceConnFactoryManager.CONN_METHOD.BLUETOOTH)
              //设置连接的蓝牙mac地址
              .setMacAddress(address)
              .build();

      //打开端口
      threadPool = ThreadPool.getInstantiation();
      threadPool.addSerialTask(new Runnable() {
        @Override
        public void run() {
          DeviceConnFactoryManager.getDeviceConnFactoryManagers().get(address).openPort();
        }
      });

      result.success(true);
    } else {
      result.error("******************* invalid_argument", "argument 'address' not found", null);
    }

  }


  private String openport(String address) {
    BluetoothDevice device = null;
    BluetoothAdapter mBluetoothAdapter = null;
    mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
    InputStream tmpIn = null;
    OutputStream tmpOut = null;
    if (mBluetoothAdapter.isEnabled()) {
      this.IsConnected = true;
      device = mBluetoothAdapter.getRemoteDevice(address);

      try {
        this.btSocket = device.createRfcommSocketToServiceRecord(MY_UUID);
      } catch (IOException var9) {
        return "-1";
      }

      mBluetoothAdapter.cancelDiscovery();

      try {
        this.btSocket.connect();
        this.OutStream = this.btSocket.getOutputStream();
        this.InStream = this.btSocket.getInputStream();
      } catch (IOException var8) {
        return "-1";
      }

      /*
      try {
        Thread.sleep(100L);
      } catch (InterruptedException var7) {
        InterruptedException e1 = var7;
        e1.printStackTrace();
      }
      */

      return "1";
    } else {
      //this.detect_bluetooth();
      this.IsConnected = false;
      return "-1";
    }
  }

  /*
  private void detect_bluetooth() {
    Toast.makeText(this, "Bluetooth not Enable", 1).show();
  }
*/

  /**
   * 关闭连接
   */
  private boolean disconnect(){
    DeviceConnFactoryManager deviceConnFactoryManager = DeviceConnFactoryManager.getDeviceConnFactoryManagers().get(curMacAddress);
    if(deviceConnFactoryManager != null && deviceConnFactoryManager.mPort != null) {
      deviceConnFactoryManager.reader.cancel();
      deviceConnFactoryManager.closePort();
      deviceConnFactoryManager.mPort = null;
    }

    return true;
  }

  private boolean destroy() {
    DeviceConnFactoryManager.closeAllPort();
    if (threadPool != null) {
      threadPool.stopThreadPool();
    }

    return true;
  }

  private void printTest(Result result) {
    final DeviceConnFactoryManager deviceConnFactoryManager = DeviceConnFactoryManager.getDeviceConnFactoryManagers().get(curMacAddress);
    if (deviceConnFactoryManager == null || !deviceConnFactoryManager.getConnState()) {
      result.error("not connect", "state not right", null);
    }

    threadPool = ThreadPool.getInstantiation();
    threadPool.addSerialTask(new Runnable() {
      @Override
      public void run() {
        assert deviceConnFactoryManager != null;
        PrinterCommand printerCommand = deviceConnFactoryManager.getCurrentPrinterCommand();

        if (printerCommand == PrinterCommand.ESC) {
          deviceConnFactoryManager.sendByteDataImmediately(FactoryCommand.printSelfTest(FactoryCommand.printerMode.ESC));
        }else if (printerCommand == PrinterCommand.TSC) {
          deviceConnFactoryManager.sendByteDataImmediately(FactoryCommand.printSelfTest(FactoryCommand.printerMode.TSC));
        }else if (printerCommand == PrinterCommand.CPCL) {
          deviceConnFactoryManager.sendByteDataImmediately(FactoryCommand.printSelfTest(FactoryCommand.printerMode.CPCL));
        }
      }
    });

  }

  @SuppressWarnings("unchecked")
  private void print(MethodCall call, Result result) {
    Map<String, Object> args = call.arguments();

    final DeviceConnFactoryManager deviceConnFactoryManager = DeviceConnFactoryManager.getDeviceConnFactoryManagers().get(curMacAddress);
    Log.e(TAG,"#### curMacAddress: " + curMacAddress);

    if (deviceConnFactoryManager == null || !deviceConnFactoryManager.getConnState()) {
      result.error("not connect", "state not right", null);
    }

    if (args != null && args.containsKey("config") && args.containsKey("data")) {
      final Map<String,Object> config = (Map<String,Object>)args.get("config");
      final List<Map<String,Object>> list = (List<Map<String,Object>>)args.get("data");
      if(list == null){
        return;
      }

      final String commandType = (String)(config.get("commandType")==null?60:config.get("commandType")); // 单位：mm

      Log.e(TAG,"#### commandType: " + commandType);

      threadPool = ThreadPool.getInstantiation();
      threadPool.addSerialTask(new Runnable() {
        @Override
        public void run() {
          assert deviceConnFactoryManager != null;
          PrinterCommand printerCommand = deviceConnFactoryManager.getCurrentPrinterCommand();

          if(!commandType.equals("TSPL")){
            if (printerCommand == PrinterCommand.ESC) {
              deviceConnFactoryManager.sendDataImmediately(PrintContent.mapToReceipt(config, list));
            }else if (printerCommand == PrinterCommand.TSC) {
              deviceConnFactoryManager.sendDataImmediately(PrintContent.mapToLabel(config, list));
            }else if (printerCommand == PrinterCommand.CPCL) {
              deviceConnFactoryManager.sendDataImmediately(PrintContent.mapToCPCL(config, list));
            }
          } else {
            Log.e(TAG, "##### printTSPL() -> ");
            printTSPL( config, list);
            //deviceConnFactoryManager.sendDataImmediately(PrintContent.mapToTSPL(config, list));
          }
          
        }
      });
    }else{
      result.error("please add config or data", "", null);
    }
  }

  @SuppressWarnings("unchecked")
  private void printTSPL(Map<String,Object> config, List<Map<String,Object>> list){

    Log.e(TAG, "##### curMacAddress -> " + curMacAddress.toString());

    int width = (int)(config.get("width")==null?50:config.get("width")); // 单位：mm
    int height = (int)(config.get("height")==null?30:config.get("height")); // 单位：mm
    int gap = (int)(config.get("gap")==null?2:config.get("gap"));
    int copyNumber = (int)(config.get("copyNumber")==null?1:config.get("copyNumber"));

    TscDll.openport(curMacAddress);
    TscDll.setup(width, height, 4, 4, 0, gap, 0);
    TscDll.sendcommand("SET TEAR ON\n");
    TscDll.sendcommand("SET COUNTER @1 1\n");
    TscDll.clearbuffer();

    for (Map<String,Object> m: list) {
      String type = (String)m.get("type");
      String content = (String)m.get("content");
      int x = (int)(m.get("x")==null?0:m.get("x"));
      int y = (int)(m.get("y")==null?0:m.get("y"));
      int size = (int)(m.get("size")==null?0:m.get("size"));

      int textwidth = (int)(m.get("width")==null?1:m.get("width"));
      int textheight = (int)(m.get("height")==null?1:m.get("height"));
      int alignment = (int)(m.get("align")==null?0:m.get("align"));

      if("text".equals(type)){
        int font = (int)(m.get("font")==null?0:m.get("font"));

        String command = "TEXT " + Integer.toString(x) + "," +
        new StringJoiner(",")
                .add(Integer.toString(y))
                .add("\"" +"0"+ "\"")
                .add("0")
                .add(Integer.toString(textwidth))
                .add(Integer.toString(textheight))
                .add(Integer.toString(alignment))
                .add("\"" + content + "\"")//.toString() ;
                .add("\r\n").toString();
        Log.e(TAG, "##### stringBuilder -> " + command);

        TscDll.sendcommand(command);

      }else if("barcode".equals(type)){
        String barcodeType = (String)(m.get("barcodeType")==""?"128":m.get("barcodeType"));
        TscDll.barcode(x, y, barcodeType, textheight, 1, 0, 3, 3, content);
      }else if("qrcode".equals(type)){
        // String message = "QRCODE " + x + "," + y + "," + ecc + "," + cell + "," + mode + "," + rotation + "," + model + "," + mask + "," + "\"" + content + "\"" + "\r\n";
        //TscDll.qrcode(x, y, "M", "5", "A", "0", "M1", "", content);
        //String message = "QRCODE " + x + "," + y + ",M,5,A,0,M1,S7," + "\"" + content + "\"";
        //QRCODE x,y,ECC Level,cell width,mode,rotation,[justification,]model,]mask,]area] "content"
        String message = qrcode(x, y, "M", "5", "A", "0", "M1", "", content);
        //TscDll.qrcode(x, y, "M", "5", "A", "0", "M1", "", content);
        Log.e(TAG, "##### QRCODE - RETURN -> " + message);
        TscDll.sendcommand(message);
      }
    }
    
    Log.e(TAG, "##### copyNumber -> " + Integer.toString(copyNumber));
    TscDll.printlabel(1, 1);
    TscDll.closeport(5000);
  }

  private String qrcode(int x, int y, String ecc, String cell, String mode, String rotation, String model, String mask, String content) {
   return "QRCODE " + x + "," + y + "," + ecc + "," + cell + "," + mode + "," + rotation + "," + model + "," + mask + "," + "\"" + content + "\"" + "\n";

  }

  @Override
  public boolean onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {

    if (requestCode == REQUEST_FINE_LOCATION_PERMISSIONS) {
      if (grantResults[0] == PackageManager.PERMISSION_GRANTED) {
        startScan(pendingCall, pendingResult);
      } else {
        pendingResult.error("no_permissions", "this plugin requires location permissions for scanning", null);
        pendingResult = null;
      }
      return true;
    }
    return false;

  }



  private final StreamHandler stateHandler = new StreamHandler() {
    private EventSink sink;

    private final BroadcastReceiver mReceiver = new BroadcastReceiver() {
      @Override
      public void onReceive(Context context, Intent intent) {
        final String action = intent.getAction();
        Log.d(TAG, "stateStreamHandler, current action: " + action);

        if (BluetoothAdapter.ACTION_STATE_CHANGED.equals(action)) {
          threadPool = null;
          sink.success(intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, -1));
        } else if (BluetoothDevice.ACTION_ACL_CONNECTED.equals(action)) {
          sink.success(1);
        } else if (BluetoothDevice.ACTION_ACL_DISCONNECTED.equals(action)) {
          threadPool = null;
          sink.success(0);
        }
      }
    };

    @Override
    public void onListen(Object o, EventSink eventSink) {
      sink = eventSink;
      IntentFilter filter = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
      filter.addAction(BluetoothAdapter.ACTION_CONNECTION_STATE_CHANGED);
      filter.addAction(BluetoothDevice.ACTION_ACL_CONNECTED);
      filter.addAction(BluetoothDevice.ACTION_ACL_DISCONNECTED);
      context.registerReceiver(mReceiver, filter);
    }

    @Override
    public void onCancel(Object o) {
      sink = null;
      context.unregisterReceiver(mReceiver);
    }
  };

}
