function doGet(e){
  
  var ss = SpreadsheetApp.getActive();

  //here in URL we need to specify sheet name which comes in the place of e.parameter["id"]
  var sheet = ss.getSheetByName(e.parameter["id"]);
  //var sheet = ss.getSheetByName('Inventory');

  //Fetching all the header as (startRowCell,startColCell, No of rows,No.of Columns)
  var headers = sheet.getRange(1, 1, 1, sheet.getLastColumn()).getValues()[0];
  var lastRow = sheet.getLastRow();

  //select A-1 cell
  var cell = sheet.getRange('a1');
  var col = 0;
  var d = new Date();
  
  for (i in headers){

    // loop through the headers and if a parameter name matches the header name insert the value

    //From URL we are manually entering all the values except timestamp hence this case is handled separately
    if (headers[i] == "Timestamp")
    {
      val = d.toDateString() + ", " + d.toLocaleTimeString();
    }
    else
    {

      val = e.parameter[headers[i]]; 
    }

    // append data to the last row
    cell.offset(lastRow, col).setValue(val);
    col++;
  }

  
  var lastRow = sheet.getLastRow();
  var status = sheet.getRange(lastRow, 10).getValue();   //select a cell (rowNo,colno)  starts from 1(not 0)
  var to = 'eyrc.vb.0000@gmail.com'

  if(status=="YES"){
      if(e.parameter["id"]=="OrdersDispatched"){
        let orderNo = sheet.getRange(lastRow, 4).getValue();
        let item = sheet.getRange(lastRow, 6).getValue();
        let qty =sheet.getRange(lastRow, 8).getValue();
        let d_t = sheet.getRange(lastRow, 11).getValue();
        let city = sheet.getRange(lastRow, 5).getValue();
        let cost = sheet.getRange(lastRow, 9).getValue();
        
        var mailBody_dispatched = `Greetings!!\n\nYour Order has been dispatched. Contact us if you have any questions. We are here to help you.\
                                  \n\nORDER SUMMARY:\nOrder Number: ${orderNo} \nItem:${item} \nQuantity: ${qty}\nDispatched Date and Time: ${d_t}\
                                  \nCity: ${city}\nCost:${cost}\n`

        MailApp.sendEmail(to, " dispatched! ", mailBody_dispatched);
        console.log("Dispatched Sent") 
    }
    else if(e.parameter["id"]=="OrdersShipped"){
        let orderNo = sheet.getRange(lastRow, 4).getValue();
        let item = sheet.getRange(lastRow, 6).getValue();
        let qty =sheet.getRange(lastRow, 8).getValue();
        let d_t = sheet.getRange(lastRow, 11).getValue();
        let city = sheet.getRange(lastRow, 5).getValue();
        let cost = sheet.getRange(lastRow, 9).getValue();
        let est =  sheet.getRange(lastRow, 12).getValue();

        var mailBody_shipped = `Greetings!!\n\nYour Order has been shipped. It will be drone delivered to you in one day. Contact us if you have\
any questions. We are here to help you.\n\nORDER SUMMARY: \nOrder Number: ${orderNo}\nItem: ${item}\nQuantity: ${qty}\ 
Shipped Date and Time: ${d_t}\nCity: ${city}\nCost:${cost}\nEstimated Time of delivery: ${est}\n`

        MailApp.sendEmail(to, " shipped! ", mailBody_shipped);
        console.log("Shipped Sent")
    }
  }
 
  return ContentService.createTextOutput('success');
}
