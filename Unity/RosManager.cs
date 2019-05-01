using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using RosSharp.RosBridgeClient;
using bm_msgs = RosSharp.RosBridgeClient.Messages.bm_message;
using bm_imsgs = RosSharp.RosBridgeClient.Messages.Sensor;
using bm_sfmsgs = RosSharp.RosBridgeClient.Messages.bm_sizefit;
using bm_fmsgs = RosSharp.RosBridgeClient.Messages.bm_fits;


public class RosManager : MonoBehaviour
{
    public GameObject uIManagerGO;

    private UIManager uiM;

    Texture2D bmp;

    public float chest_size = 0;
    public float waist_size = 0;
    public float bottom_size = 0;
    public float shoulder_size = 0;
    public float sleeve_size = 0;

    byte[] _displayPixels = new byte[424 * 512 * 4];

    int check_flag = 0;

    string rosBMSizeFit_Id;

    int chest_fit = 3;
    int waist_fit = 3;
    int bottom_fit = 3;


    RosSocket rosSocket = null;

    bm_sfmsgs sizefit_message = new bm_sfmsgs();

    // Start is called before the first frame update
    void Start()
    {
        uiM = uIManagerGO.GetComponent<UIManager>();

        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol("ws://192.168.189.130:9090"));

        rosBMSizeFit_Id = rosSocket.Advertise<bm_sfmsgs>("/bm_SizeFitChatter");

        string rosBMMeasurements_Id = rosSocket.Subscribe<bm_msgs>("/bm_MeasurementsChatter", MeasurementsSubscriptionHandler);
        //rosBMMeasurements_Id = rosSocket.Subscribe<bm_msgs>("/bm_MeasurementsChatter", MeasurementsSubscriptionHandler);

        string rosBMImage_Id = rosSocket.Subscribe<bm_imsgs.Image>("/bm_ImagesChatter", ImageSubscriptionHandler);
        //rosBMImage_Id = rosSocket.Subscribe<bm_imsgs.Image>("/bm_ImagesChatter", ImageSubscriptionHandler);

        string rosBMFits_Id = rosSocket.Subscribe<bm_fmsgs>("/bm_FitsChatter", FitsSubscriptionHandler);

        //change Pinda's chest color for test
        //pindaChest.GetComponent<MeshRenderer>().material = blueMaterial;
        changePindaColor(3, 3, 3);

    }

    private void MeasurementsSubscriptionHandler(bm_msgs measurements_message)
    {

        Debug.Log((measurements_message).chest_size_msg);
        Debug.Log((measurements_message).waist_size_msg);
        Debug.Log((measurements_message).bottom_size_msg);

        chest_size = (measurements_message).chest_size_msg;
        waist_size = (measurements_message).waist_size_msg;
        bottom_size = (measurements_message).bottom_size_msg;
        shoulder_size = (measurements_message).shoulder_size_msg;
        sleeve_size = (measurements_message).sleeve_size_msg;

        check_flag = 1;
    }

    private void ImageSubscriptionHandler(bm_imsgs.Image image_message)
    {
        Debug.Log("Image Arrived");
        _displayPixels = image_message.data;

        //Texture2D tex = new Texture2D(2, 2);
        //bmp = new Texture2D(512, 424, TextureFormat.RGBA32, false);
        //bmp.LoadRawTextureData(_displayPixels);
        //bmp.Apply();
        ////uiM.rawPersonImage.texture = bmp;

        check_flag = 1;
    }

    private void FitsSubscriptionHandler(bm_fmsgs fits_message)
    {
        Debug.Log("Fits Arrived" + fits_message.chest_fit_msg);

        chest_fit = fits_message.chest_fit_msg;
        waist_fit = fits_message.waist_fit_msg;
        bottom_fit = fits_message.bottom_fit_msg;


        check_flag = 1;
    }

    // Update is called once per frame
    void Update()
    {
        if (check_flag == 1)
        {
            uiM.chestSize.text = chest_size.ToString();
            uiM.waistSize.text = waist_size.ToString();
            uiM.bottomSize.text = bottom_size.ToString();
            uiM.shoulderSize.text = shoulder_size.ToString();
            uiM.sleeveSize.text = sleeve_size.ToString();

            Texture2D tex = new Texture2D(2, 2);
            Texture2D bmp = new Texture2D(512, 424, TextureFormat.RGBA32, false);
            bmp.LoadRawTextureData(_displayPixels);
            bmp.Apply();
            uiM.rawPersonImage.texture = bmp;

            changePindaColor(chest_fit, waist_fit, bottom_fit);

            tex.LoadImage(_displayPixels);

            changePindaColor(chest_fit, waist_fit, bottom_fit);


            //uiM.personImage.sprite.m = tex;
            //Debug.Log(_displayPixels.Length);

            check_flag = 0;
        }

    }

    
   public void changePindaColor(int chestFit, int waistFit, int bottomFit)
    {

        Material waistFitColor = colorPicker(waistFit);
        Material bottomFitColor = colorPicker(bottomFit);
        Material chestFitColor = colorPicker(chestFit);


        uiM.pindaWaist.GetComponent<MeshRenderer>().material = waistFitColor;
        uiM.pindaBottom.GetComponent<MeshRenderer>().material = bottomFitColor;
        uiM.pindaChest.GetComponent<MeshRenderer>().material = chestFitColor;
    }

    public Material colorPicker(int integerFit)
    {
        
        if (integerFit == 1)
        {
            return uiM.purpleMaterial;
        }
        else if (integerFit == 2)
        {
            return uiM.blueMaterial;
        }
        else if (integerFit == 3)
        {
            return uiM.greenMaterial;
        }
        else if (integerFit == 4)
        {
            return uiM.orangeMaterial;
        }
        else if (integerFit == 5)
        {
            return uiM.redMaterial;
        }
        else
        {
            return uiM.greenMaterial;
        }
    }
    public void SizeDropdownChange()
    {
        Debug.Log("Size Dropdown Value changed to " + uiM.GetSizeDropDownValue());

        sizefit_message.size_msg = uiM.GetSizeDropDownValue();

    }

    public void FitDropdownChange()
    {
        Debug.Log("Fit Dropdown Value changed to " + uiM.GetFitDropDownValue());

        sizefit_message.fit_msg = uiM.GetFitDropDownValue();

    }

    //callback when proceed button is pressed
    public void ProceedButtonPressed()
    {
        Debug.Log("Proceed Button Pressed! ");

        rosSocket.Publish(rosBMSizeFit_Id, sizefit_message);
    }
}
