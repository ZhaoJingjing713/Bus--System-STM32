// pages/map/map.js
Page({
 
  /**
   * 地图页面的初始数据
   */
  data: {
    num:0,
    longitude:121.522340,
    latitude:38.883508,
    markers: [{ // 绘制浮标，传入JSON支持多个
      iconPath: "../../images/01.png", //浮标图片路径，推荐png图片
      id: 0, // Id支持多个，方便后期点击浮标获取相关信息
      latitude: 38.883508, // 经度
      longitude: 121.522340, //纬度
      width: 50, // 浮标宽度
      height: 50 // 浮标高度
    }],
    polyline: [{ // 绘制多边形区域，两个点绘制直线
      points: [ // 这里传入两个点是直线，如果传入三个点以上就会是首尾相连的多边形区域
        {
          longitude: 121.532102,
          latitude: 38.889807
        },
        {
          longitude: 121.53106,
          latitude: 38.889849
        }],
        color: "#FF0000DD", // 设置颜色
        width: 2, // 设置线宽度 注：电脑模拟器无法预览测设设置，此设置需要手机测试
        dottedLine: true // 是否设置为虚线
    }],
  },
  markertap(e) { // 这是一个事件，在wxml中绑定这个事件，点击浮标后
    wx.openLocation({ //此设置属于高级APi,可以打开微信内置地图组件
      latitude: 38.888455,
      longitude: 121.669802,
    });
  },
 
  /**
   * 生命周期函数--监听页面加载
   */
  onLoad: function (options) {
    var that=this;
    setInterval(function () {
      wx.request({
        url:"https://class.dlut-elab.com/feedback/mayiming/GPS.php",
        method:"POST",
        header:{
          "Content-type": "application/x-www-form-urlencoded"
        },
        data:{
          
        },
        success: function(res){
          console.log(res.data);
          that.setData({
            num:res.data[0].num,
            latitude:res.data[0].N,
            longitude:res.data[0].E,
            markers: [{ // 绘制浮标，传入JSON支持多个
              iconPath: "../../images/01.png", //浮标图片路径，推荐png图片
              id: 0, // Id支持多个，方便后期点击浮标获取相关信息
              longitude: res.data[0].E, // 经度
              latitude: res.data[0].N, //纬度
              width: 50, // 浮标宽度
              height: 50 // 浮标高度
            }]
          })
        },
        fail:function(res){
          console.log(res);
        }
      })//循环执行代码
      }, 1000) //循环时间 这里是1秒 
  },
 
  /**
   * 生命周期函数--监听页面初次渲染完成
   */
  onReady: function () {
 
  },
 
  /**
   * 生命周期函数--监听页面显示
   */
  onShow: function () {
 
  },
 
  /**
   * 生命周期函数--监听页面隐藏
   */
  onHide: function () {
 
  },
 
  /**
   * 生命周期函数--监听页面卸载
   */
  onUnload: function () {
 
  },
 
  /**
   * 页面相关事件处理函数--监听用户下拉动作
   */
  onPullDownRefresh: function () {
 
  },
 
  /**
   * 页面上拉触底事件的处理函数
   */
  onReachBottom: function () {
 
  },
 
  /**
   * 用户点击右上角分享
   */
  onShareAppMessage: function () {
 
  }
})